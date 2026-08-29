#include "release_manifest_audit/audit.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <fcntl.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <openssl/evp.h>
#include <sys/stat.h>
#include <unistd.h>

namespace release_manifest_audit
{
namespace
{
namespace fs = std::filesystem;

struct Json;
using Object = std::map<std::string, Json>;
using Array = std::vector<Json>;
struct Json : std::variant<std::nullptr_t, bool, double, std::string, Array, Object>
{
  using variant::variant;
};

class Parser
{
public:
  explicit Parser(const std::string & input) : input_(input) {}
  Json parse()
  {
    const auto value = value_at();
    whitespace();
    if (position_ != input_.size()) { throw std::runtime_error("trailing JSON data"); }
    return value;
  }
private:
  void whitespace()
  {
    while (position_ < input_.size() && std::isspace(static_cast<unsigned char>(input_[position_]))) { ++position_; }
  }
  char take()
  {
    if (position_ == input_.size()) { throw std::runtime_error("unexpected end of JSON"); }
    return input_[position_++];
  }
  void expect(char expected)
  {
    whitespace();
    if (take() != expected) { throw std::runtime_error("invalid JSON punctuation"); }
  }
  Json value_at()
  {
    whitespace();
    if (position_ == input_.size()) { throw std::runtime_error("missing JSON value"); }
    switch (input_[position_]) {
      case '{': return object(); case '[': return array(); case '"': return string();
      case 't': literal("true", true); return true; case 'f': literal("false", false); return false;
      case 'n': literal("null", false); return nullptr; default: return number();
    }
  }
  void literal(const char * text, bool)
  {
    for (const char * cursor = text; *cursor != '\0'; ++cursor) { if (take() != *cursor) { throw std::runtime_error("invalid JSON literal"); } }
  }
  std::string string()
  {
    expect('"'); std::string result;
    while (true) {
      const char value = take();
      if (value == '"') { return result; }
      if (static_cast<unsigned char>(value) < 0x20U) { throw std::runtime_error("control byte in JSON string"); }
      if (value != '\\') { result += value; continue; }
      switch (take()) {
        case '"': result += '"'; break; case '\\': result += '\\'; break; case '/': result += '/'; break;
        case 'b': result += '\b'; break; case 'f': result += '\f'; break; case 'n': result += '\n'; break;
        case 'r': result += '\r'; break; case 't': result += '\t'; break;
        case 'u': {
          unsigned value_u = 0U;
          for (int i = 0; i < 4; ++i) { const char hex = take(); if (!std::isxdigit(static_cast<unsigned char>(hex))) { throw std::runtime_error("invalid JSON unicode escape"); } value_u = value_u * 16U + static_cast<unsigned>(std::isdigit(hex) ? hex - '0' : std::tolower(hex) - 'a' + 10); }
          if (value_u > 0x7fU) { throw std::runtime_error("non-ASCII unicode escapes are not accepted by this audit parser"); }
          result += static_cast<char>(value_u); break;
        }
        default: throw std::runtime_error("invalid JSON escape");
      }
    }
  }
  Json number()
  {
    const auto begin = position_;
    if (input_[position_] == '-') { ++position_; }
    while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) { ++position_; }
    if (position_ < input_.size() && input_[position_] == '.') { ++position_; while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) { ++position_; } }
    if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) { ++position_; if (position_ < input_.size() && (input_[position_] == '+' || input_[position_] == '-')) { ++position_; } while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) { ++position_; } }
    const auto text = input_.substr(begin, position_ - begin); char * end = nullptr; errno = 0;
    const double result = std::strtod(text.c_str(), &end);
    if (text.empty() || end == nullptr || *end != '\0' || errno == ERANGE || !std::isfinite(result)) { throw std::runtime_error("invalid JSON number"); }
    return result;
  }
  Json array()
  {
    expect('['); Array result; whitespace(); if (position_ < input_.size() && input_[position_] == ']') { ++position_; return result; }
    while (true) { result.push_back(value_at()); whitespace(); const char separator = take(); if (separator == ']') { return result; } if (separator != ',') { throw std::runtime_error("invalid JSON array"); } }
  }
  Json object()
  {
    expect('{'); Object result; whitespace(); if (position_ < input_.size() && input_[position_] == '}') { ++position_; return result; }
    while (true) { whitespace(); if (position_ == input_.size() || input_[position_] != '"') { throw std::runtime_error("object key must be a string"); } const auto key = string(); expect(':'); if (!result.emplace(key, value_at()).second) { throw std::runtime_error("duplicate JSON object key: " + key); } whitespace(); const char separator = take(); if (separator == '}') { return result; } if (separator != ',') { throw std::runtime_error("invalid JSON object"); } }
  }
  const std::string & input_; std::size_t position_{0U};
};

std::string text(const Json & value, const std::string & context)
{ if (!std::holds_alternative<std::string>(value)) { throw std::runtime_error(context + " must be a string"); } return std::get<std::string>(value); }
bool boolean(const Json & value, const std::string & context)
{ if (!std::holds_alternative<bool>(value)) { throw std::runtime_error(context + " must be a boolean"); } return std::get<bool>(value); }
const Object & object(const Json & value, const std::string & context)
{ if (!std::holds_alternative<Object>(value)) { throw std::runtime_error(context + " must be an object"); } return std::get<Object>(value); }
const Array & array(const Json & value, const std::string & context)
{ if (!std::holds_alternative<Array>(value)) { throw std::runtime_error(context + " must be an array"); } return std::get<Array>(value); }
const Json * find(const Object & value, const std::string & key)
{ const auto it = value.find(key); return it == value.end() ? nullptr : &it->second; }
const Json & required(const Object & value, const std::string & key, const std::string & context)
{ const auto * result = find(value, key); if (result == nullptr) { throw std::runtime_error(context + " missing required field '" + key + "'"); } return *result; }
std::optional<std::string> optional_text(const Object & value, const std::string & key)
{ const auto * item = find(value, key); return item == nullptr ? std::nullopt : std::optional<std::string>(text(*item, key)); }

std::string read_file(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary); if (!input) { throw std::runtime_error("cannot read input: " + path.string()); }
  std::ostringstream output; output << input.rdbuf(); if (input.bad()) { throw std::runtime_error("cannot fully read input: " + path.string()); } return output.str();
}
std::string sha256(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary); if (!input) { throw std::runtime_error("cannot hash input: " + path.string()); }
  EVP_MD_CTX * context = EVP_MD_CTX_new(); if (!context || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) { EVP_MD_CTX_free(context); throw std::runtime_error("cannot initialize SHA-256"); }
  char buffer[65536]; while (input) { input.read(buffer, sizeof(buffer)); const auto count = input.gcount(); if (count > 0 && EVP_DigestUpdate(context, buffer, static_cast<std::size_t>(count)) != 1) { EVP_MD_CTX_free(context); throw std::runtime_error("cannot calculate SHA-256"); } }
  if (!input.eof()) { EVP_MD_CTX_free(context); throw std::runtime_error("cannot read input for SHA-256: " + path.string()); }
  unsigned char digest[EVP_MAX_MD_SIZE]{}; unsigned size = 0U; if (EVP_DigestFinal_ex(context, digest, &size) != 1) { EVP_MD_CTX_free(context); throw std::runtime_error("cannot finalize SHA-256"); } EVP_MD_CTX_free(context);
  std::ostringstream output; output << std::hex << std::setfill('0'); for (unsigned i = 0; i < size; ++i) { output << std::setw(2) << static_cast<unsigned>(digest[i]); } return output.str();
}
bool sha256_text(const std::string & value)
{ return value.size() == 64U && std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isxdigit(c) != 0; }); }
bool git_sha(const std::string & value)
{ return value.size() == 40U && std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isxdigit(c) != 0; }); }
std::string escape(const std::string & value)
{ std::ostringstream out; for (unsigned char c : value) { if (c == '"') out << "\\\""; else if (c == '\\') out << "\\\\"; else if (c == '\n') out << "\\n"; else if (c == '\r') out << "\\r"; else if (c == '\t') out << "\\t"; else if (c < 32U) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(c) << std::dec; else out << static_cast<char>(c); } return out.str(); }
std::string render(const Json & value)
{
  if (std::holds_alternative<std::nullptr_t>(value)) return "null";
  if (std::holds_alternative<bool>(value)) return std::get<bool>(value) ? "true" : "false";
  if (std::holds_alternative<double>(value)) { std::ostringstream out; out.imbue(std::locale::classic()); out << std::setprecision(17) << std::get<double>(value); return out.str(); }
  if (std::holds_alternative<std::string>(value)) return "\"" + escape(std::get<std::string>(value)) + "\"";
  if (std::holds_alternative<Array>(value)) { std::ostringstream out; out << '['; const auto & values = std::get<Array>(value); for (std::size_t i = 0; i < values.size(); ++i) { if (i) out << ','; out << render(values[i]); } return out << ']', out.str(); }
  std::ostringstream out; out << '{'; bool first = true; for (const auto & [key, item] : std::get<Object>(value)) { if (!first) out << ','; first = false; out << '"' << escape(key) << "\":" << render(item); } out << '}'; return out.str();
}
void write_new(const fs::path & path, const std::string & contents)
{
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (descriptor < 0) { throw std::runtime_error("refusing to overwrite output: " + path.string()); }
  std::size_t done = 0U; while (done < contents.size()) { const auto written = ::write(descriptor, contents.data() + done, contents.size() - done); if (written <= 0) { ::close(descriptor); throw std::runtime_error("cannot write output: " + path.string()); } done += static_cast<std::size_t>(written); }
  if (::close(descriptor) != 0) throw std::runtime_error("cannot close output: " + path.string());
}

enum class Status { Pass, Fail, Unavailable, NotRun, NotVerified };
const char * name(Status value) { switch (value) { case Status::Pass: return "PASS"; case Status::Fail: return "FAIL"; case Status::Unavailable: return "UNAVAILABLE"; case Status::NotRun: return "NOT_RUN"; case Status::NotVerified: return "NOT_VERIFIED"; } return "FAIL"; }
Status status(const std::string & value)
{ if (value == "PASS") return Status::Pass; if (value == "FAIL") return Status::Fail; if (value == "UNAVAILABLE") return Status::Unavailable; if (value == "NOT_RUN") return Status::NotRun; if (value == "NOT_VERIFIED") return Status::NotVerified; throw std::runtime_error("unknown status: " + value); }
bool is_absence_status(Status value)
{ return value == Status::Unavailable || value == Status::NotRun || value == Status::NotVerified; }

std::string lower_token(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const auto character : value) {
    result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
  return result;
}

std::string trim_token(const std::string & value)
{
  std::size_t begin = 0U;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1U])) != 0) {
    --end;
  }
  return value.substr(begin, end - begin);
}

// Normalize human-written keys as well as values. Reports may use snake_case,
// kebab-case, or camelCase spellings; safety claims must not become invisible
// merely because a producer changed separators or capitalization.
std::string canonical_key(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  bool previous_alnum = false;
  bool previous_lower_or_digit = false;
  for (const auto character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if (std::isalnum(byte) != 0) {
      const bool upper = std::isupper(byte) != 0;
      if (upper && previous_lower_or_digit && !result.empty() && result.back() != '_') {
        result.push_back('_');
      }
      result.push_back(static_cast<char>(std::tolower(byte)));
      previous_alnum = true;
      previous_lower_or_digit = !upper;
    } else {
      if (previous_alnum && !result.empty() && result.back() != '_') result.push_back('_');
      previous_alnum = false;
      previous_lower_or_digit = false;
    }
  }
  while (!result.empty() && result.back() == '_') result.pop_back();
  return result;
}

// Claim keys are compared independent of case, separators, or camel-case
// boundaries.  This keeps aliases such as production_ready, production-ready,
// ProductionReady, and PRODUCTIONREADY in the same deny-list namespace.
std::string canonical_claim_key(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const auto character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if (std::isalnum(byte) != 0) {
      result.push_back(static_cast<char>(std::tolower(byte)));
    }
  }
  return result;
}

bool json_bool(const Object & value, const char * key, bool * present = nullptr)
{
  const auto * item = find(value, key);
  if (present != nullptr) *present = item != nullptr;
  return item != nullptr && std::holds_alternative<bool>(*item) && std::get<bool>(*item);
}

std::optional<double> json_number(const Object & value, const char * key)
{
  const auto * item = find(value, key);
  if (!item || !std::holds_alternative<double>(*item)) return std::nullopt;
  return std::get<double>(*item);
}

std::string canonical_kind(const std::string & value)
{
  return canonical_key(value);
}

const std::set<std::string> & allowed_kinds()
{
  static const std::set<std::string> values{
    "build", "ctest", "release_smoke", "ros_e2e", "ros_message_e2e",
    "scenario_benchmark", "scenario", "model_qualification", "qualification",
    "calibration", "evidence", "evidence_report", "evidence_bundle", "bundle",
    "orin_preflight", "test", "model", "production_model", "formal_model",
    "formal_calibration", "camera", "orin", "cdc", "gimbal", "firing", "hardware",
  };
  return values;
}

bool is_hardware_kind(const std::string & value)
{
  static const std::set<std::string> values{
    "model", "production_model", "formal_model", "formal_calibration", "camera", "orin",
    "cdc", "gimbal", "firing", "hardware",
  };
  return values.count(value) != 0U;
}

bool has_symlink_component(const fs::path & path)
{
  fs::path current;
  for (const auto & component : path) {
    if (component == "/") {
      current = component;
      continue;
    }
    current /= component;
    std::error_code error;
    const auto state = fs::symlink_status(current, error);
    if (!error && fs::is_symlink(state)) return true;
  }
  return false;
}

bool regular_nonsymlink(const fs::path & path, std::uintmax_t * size = nullptr)
{
  if (has_symlink_component(path)) return false;
  std::error_code error;
  const auto state = fs::symlink_status(path, error);
  if (error || !fs::is_regular_file(state)) return false;
  if (size != nullptr) {
    *size = fs::file_size(path, error);
    if (error) return false;
  }
  return true;
}

std::optional<Status> parse_status_value(const Json & value, const std::string & context,
                                         std::vector<std::string> * reasons)
{
  if (!std::holds_alternative<std::string>(value)) {
    reasons->push_back(context + " must be a string status");
    return std::nullopt;
  }
  const auto raw = lower_token(trim_token(std::get<std::string>(value)));
  if (raw == "warn") return Status::NotVerified;
  if (raw == "ready_candidate") return Status::Pass;
  if (raw == "blocked") return Status::Fail;
  try {
    return status([&]() {
      std::string upper;
      upper.reserve(raw.size());
      for (const auto character : raw) upper.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(character))));
      return upper;
    }());
  } catch (const std::exception &) {
    reasons->push_back(context + " has unknown status");
    return std::nullopt;
  }
}

std::optional<Status> status_hint(const Object & root, std::vector<std::string> * reasons,
                                  std::string * raw_hint = nullptr)
{
  static const std::set<std::string> aliases{
    "status", "overallstatus", "result", "softwarecandidatestatus",
    "reportstatus", "bundlestatus", "bundlereportstatus"};
  std::optional<Status> selected;
  for (const auto & [key, value] : root) {
    if (aliases.count(canonical_claim_key(key)) == 0U) continue;
    const auto parsed = parse_status_value(value, key, reasons);
    if (!parsed) continue;
    if (raw_hint != nullptr && !raw_hint->empty()) {
      // Keep the first raw value for diagnostics; all aliases are still checked.
    } else if (raw_hint != nullptr && std::holds_alternative<std::string>(value)) {
      *raw_hint = std::get<std::string>(value);
    }
    if (!selected) {
      selected = parsed;
    } else if (*selected != *parsed) {
      reasons->push_back("status aliases disagree");
    }
  }
  if (!selected) reasons->push_back("missing report status");
  return selected;
}
std::optional<Status> absence_status(const Object & value, std::vector<std::string> * reasons, const std::string & context)
{
  const auto declared = optional_text(value, "absence_status");
  if (!declared) return std::nullopt;
  try {
    const auto result = status(*declared);
    if (is_absence_status(result)) return result;
  } catch (const std::exception &) {
  }
  reasons->push_back(context + " absence_status must be UNAVAILABLE, NOT_RUN, or NOT_VERIFIED");
  return Status::Fail;
}
bool unsafe_claim(const Json & value, bool allow_negative_production_claim = false,
                  bool top_level = true)
{
  if (std::holds_alternative<Object>(value)) {
    for (const auto & [key, child] : std::get<Object>(value)) {
      // A negative evidence fixture may intentionally contain exactly one
      // top-level production_ready=true claim: that is the claim whose
      // rejection is being demonstrated.  Every nested/aliased claim remains
      // unsafe and therefore fatal.
      const bool expected_negative_claim = allow_negative_production_claim &&
        top_level && key == "production_ready" && std::holds_alternative<bool>(child) &&
        std::get<bool>(child);
      const auto lower = canonical_claim_key(key);
      const auto bool_value = [&]() -> std::optional<bool> {
        if (std::holds_alternative<bool>(child)) return std::get<bool>(child);
        if (std::holds_alternative<std::string>(child)) {
          const auto text_value = lower_token(trim_token(std::get<std::string>(child)));
          if (text_value == "true" || text_value == "yes" || text_value == "1") return true;
          if (text_value == "false" || text_value == "no" || text_value == "0") return false;
        }
        return std::nullopt;
      }();
      const auto production_flag = lower == "productionready" || lower == "hardwarevalidation" ||
        lower == "firingvalidated" || lower == "gimbalclosedloopvalidated" ||
        lower == "realhitratecomputed" || lower == "hardwareclaim" ||
        lower == "serialenabled" || lower == "allowfire" ||
        lower == "absolutecommandvalid" || lower == "relativeangleasrobotctrl" ||
        lower == "relativeangleusedasrobotctrl" || lower == "robotctrlabsoluteangle" ||
        (lower.find("productionready") != std::string::npos) ||
        (lower.find("hardwarevalidation") != std::string::npos) ||
        (lower.find("firingvalidated") != std::string::npos) ||
        (lower.find("closedloopvalidated") != std::string::npos) ||
        (lower.find("controlapplied") != std::string::npos);
      if (!expected_negative_claim) {
        if (production_flag && bool_value && *bool_value) return true;
        if ((lower == "testonly" || lower == "dryrun") && bool_value && !*bool_value) return true;
        if (lower == "motionnonzero" && (!bool_value || *bool_value)) return true;
        const auto motion_field = lower.find("fire") != std::string::npos ||
          lower.find("velocity") != std::string::npos || lower.find("acceleration") != std::string::npos ||
          lower == "yawvel" || lower == "pitchvel" || lower == "yawacc" || lower == "pitchacc";
        if (motion_field) {
          if (std::holds_alternative<double>(child) && std::get<double>(child) != 0.0) return true;
          if (!std::holds_alternative<double>(child) && !bool_value) return true;
        }
      }
      if (unsafe_claim(child, allow_negative_production_claim, false)) return true;
    }
  }
  if (std::holds_alternative<Array>(value)) {
    for (const auto & child : std::get<Array>(value)) {
      if (unsafe_claim(child, allow_negative_production_claim, false)) return true;
    }
  }
  return false;
}
bool safe_fields(const Object & root, std::string * why)
{
  const Object * safety = &root; if (const auto * nested = find(root, "safety_assertions")) safety = &object(*nested, "safety_assertions"); else if (const auto * nested = find(root, "safety")) safety = &object(*nested, "safety"); else if (const auto * nested = find(root, "safety_defaults")) safety = &object(*nested, "safety_defaults");
  const std::vector<std::pair<std::string, bool>> values{{"serial_enabled", false}, {"dry_run", true}, {"allow_fire", false}};
  for (const auto & [key, expected] : values) { const auto * item = find(*safety, key); if (!item || !std::holds_alternative<bool>(*item) || std::get<bool>(*item) != expected) { *why = "unsafe or missing " + key; return false; } }
  const std::array<std::pair<const char *, const char *>, 5> numeric{{
    {"fire_command", "fire_command"}, {"yaw_vel", "yaw_vel_rad_s"},
    {"pitch_vel", "pitch_vel_rad_s"}, {"yaw_acc", "yaw_acc_rad_s2"},
    {"pitch_acc", "pitch_acc_rad_s2"}}};
  for (const auto & [key, alias] : numeric) {
    const auto * item = find(*safety, key);
    const auto * alias_item = find(*safety, alias);
    if (!item) item = alias_item;
    if (!item || !std::holds_alternative<double>(*item) || std::get<double>(*item) != 0.0) { *why = "unsafe or missing " + std::string(key); return false; }
    if (alias_item && (!std::holds_alternative<double>(*alias_item) || std::get<double>(*alias_item) != 0.0)) { *why = "unsafe or missing " + std::string(alias); return false; }
    if (find(*safety, key) && alias_item && std::get<double>(*find(*safety, key)) != std::get<double>(*alias_item)) { *why = "safety aliases disagree for " + std::string(key); return false; }
  }
  return true;
}
bool expected_synthetic(const Object & root, std::string * why,
                        bool allow_negative_production_claim = false)
{
  for (const auto & [key, expected] : std::vector<std::pair<std::string, bool>>{{"synthetic", true}, {"test_only", true}, {"production_ready", false}}) {
    const auto * item = find(root, key);
    if (allow_negative_production_claim && key == "production_ready" && item &&
      std::holds_alternative<bool>(*item) && std::get<bool>(*item)) continue;
    if (!item || !std::holds_alternative<bool>(*item) || std::get<bool>(*item) != expected) {
      *why = "missing or contradictory " + key;
      return false;
    }
  }
  return true;
}
bool counts_valid(const Object & root, std::string * why)
{
  const auto * counts_item = find(root, "counts"); const auto * cases_item = find(root, "cases"); if (!counts_item || !cases_item) return true;
  const auto & counts = object(*counts_item, "counts"); const auto & cases = array(*cases_item, "cases"); std::map<std::string, int> actual{{"PASS",0},{"FAIL",0},{"UNAVAILABLE",0},{"NOT_RUN",0},{"NOT_VERIFIED",0}};
  if (cases.empty()) { *why = "report cases must not be empty"; return false; }
  for (const auto & [key, unused] : counts) {
    (void)unused;
    if (actual.count(key) == 0U) {
      *why = "counts contains unknown status key: " + key;
      return false;
    }
  }
  for (const auto & item : cases) { const auto & entry = object(item, "case"); const auto value = text(required(entry, "status", "case"), "case.status"); if (!actual.count(value)) { *why = "case has unknown status"; return false; } ++actual[value]; }
  for (const auto & [key, total] : actual) { const auto * item = find(counts, key); if (!item || !std::holds_alternative<double>(*item) || std::get<double>(*item) != total) { *why = "counts disagree with cases for " + key; return false; } } return true;
}
bool status_matches_results(const Object & root, Status reported, std::string * why)
{
  const auto * counts_item = find(root, "counts"); const auto * cases_item = find(root, "cases");
  if (!counts_item && !cases_item) return true;
  if (!counts_item || !cases_item) { *why = "top-level status cannot be verified without both counts and cases"; return false; }
  const auto & counts = object(*counts_item, "counts");
  std::map<std::string, int> totals;
  for (const auto & key : {"PASS", "FAIL", "UNAVAILABLE", "NOT_RUN", "NOT_VERIFIED"}) {
    const auto * item = find(counts, key);
    if (!item || !std::holds_alternative<double>(*item)) { *why = "top-level status cannot be verified from counts"; return false; }
    totals[key] = static_cast<int>(std::get<double>(*item));
  }
  Status aggregate = Status::Pass;
  if (totals["FAIL"] > 0) aggregate = Status::Fail;
  else if (totals["NOT_VERIFIED"] > 0) aggregate = Status::NotVerified;
  else if (totals["NOT_RUN"] > 0) aggregate = Status::NotRun;
  else if (totals["UNAVAILABLE"] > 0) aggregate = Status::Unavailable;
  if (reported != aggregate) { *why = "top-level status " + std::string(name(reported)) + " contradicts aggregated case status " + name(aggregate); return false; }
  return true;
}
bool warn_matches_results(const Object & root, std::string * why)
{
  const auto * counts_item = find(root, "counts"); const auto * cases_item = find(root, "cases");
  if (!counts_item && !cases_item) return true;
  if (!counts_item || !cases_item) { *why = "top-level WARN cannot be verified without both counts and cases"; return false; }
  const auto & counts = object(*counts_item, "counts");
  const auto * failures = find(counts, "FAIL");
  if (!failures || !std::holds_alternative<double>(*failures)) { *why = "top-level WARN cannot be verified from counts"; return false; }
  if (std::get<double>(*failures) != 0.0) { *why = "top-level WARN contradicts FAIL results"; return false; }
  // A warning-shaped report is still an explicit non-PASS outcome.  Require
  // all five counters to be present and consistent so a producer cannot hide
  // an unknown status key or omit the cases used to derive the warning.
  const auto & cases = array(*cases_item, "cases");
  const auto expected_keys = std::array<const char *, 5>{{"PASS", "FAIL", "UNAVAILABLE", "NOT_RUN", "NOT_VERIFIED"}};
  for (const auto key : expected_keys) {
    const auto * item = find(counts, key);
    if (!item || !std::holds_alternative<double>(*item) || std::get<double>(*item) < 0.0) {
      *why = "top-level WARN cannot be verified from counts"; return false;
    }
  }
  if (static_cast<double>(cases.size()) !=
    std::get<double>(*find(counts, "PASS")) + std::get<double>(*find(counts, "FAIL")) +
    std::get<double>(*find(counts, "UNAVAILABLE")) + std::get<double>(*find(counts, "NOT_RUN")) +
    std::get<double>(*find(counts, "NOT_VERIFIED"))) {
    *why = "top-level WARN counts disagree with cases"; return false;
  }
  return true;
}
struct CTestRecord { std::string name; std::string status; };
struct CTestCounts { int total{0}; int passed{0}; int failed{0}; int skipped{0}; std::vector<CTestRecord> records; };
bool ctest_counts(const fs::path & path, CTestCounts * result, std::string * why)
{
  if (!regular_nonsymlink(path)) { *why = "CTest XML is not a regular non-link file"; return false; }
  const auto document = std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)>(
    xmlReadFile(path.c_str(), nullptr, XML_PARSE_NONET | XML_PARSE_NOBLANKS | XML_PARSE_NOERROR | XML_PARSE_NOWARNING),
    &xmlFreeDoc);
  if (!document) { *why = "CTest XML cannot be read or is malformed"; return false; }
  const auto root = xmlDocGetRootElement(document.get());
  if (!root || xmlStrcmp(root->name, BAD_CAST "Site") != 0) { *why = "CTest XML root must be Site"; return false; }
  xmlNode * testing = nullptr;
  for (auto node = root->children; node != nullptr; node = node->next) {
    if (node->type == XML_ELEMENT_NODE && xmlStrcmp(node->name, BAD_CAST "Testing") == 0) {
      if (testing != nullptr) { *why = "CTest XML contains multiple Testing elements"; return false; }
      testing = node;
    }
  }
  if (!testing) { *why = "CTest XML has no Testing element"; return false; }
  for (auto node = testing->children; node != nullptr; node = node->next) {
    if (node->type != XML_ELEMENT_NODE || xmlStrcmp(node->name, BAD_CAST "Test") != 0) continue;
    const auto attribute = std::unique_ptr<xmlChar, decltype(xmlFree)>(xmlGetProp(node, BAD_CAST "Status"), xmlFree);
    if (!attribute) { *why = "CTest result Test record has no Status attribute"; return false; }
    std::string value(reinterpret_cast<const char *>(attribute.get()));
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::string test_name;
    for (auto child = node->children; child != nullptr; child = child->next) {
      if (child->type == XML_ELEMENT_NODE && xmlStrcmp(child->name, BAD_CAST "Name") == 0) {
        const auto name_value = std::unique_ptr<xmlChar, decltype(xmlFree)>(
          xmlNodeGetContent(child), xmlFree);
        if (name_value) test_name = reinterpret_cast<const char *>(name_value.get());
        break;
      }
    }
    if (test_name.empty()) { *why = "CTest result Test record has no Name"; return false; }
    if (std::any_of(result->records.begin(), result->records.end(),
      [&](const CTestRecord & item) { return item.name == test_name; }))
    {
      *why = "CTest XML contains duplicate test name: " + test_name;
      return false;
    }
    ++result->total;
    std::string canonical_status;
    if (value == "passed") { ++result->passed; canonical_status = "PASS"; }
    else if (value == "failed") { ++result->failed; canonical_status = "FAIL"; }
    else if (value == "notrun" || value == "not_run" || value == "skipped") { ++result->skipped; canonical_status = "NOT_RUN"; }
    else { *why = "CTest Test record has unsupported Status: " + value; return false; }
    result->records.push_back({test_name, canonical_status});
  }
  if (result->total == 0) { *why = "CTest XML contains no Test result records with Status"; return false; }
  return true;
}
bool ctest_matches_report(const Object & root, const fs::path & path, std::string * why)
{
  const auto * counts_item = find(root, "counts"); const auto * cases_item = find(root, "cases");
  if (!counts_item || !cases_item) { *why = "CTest-backed report must provide both counts and cases"; return false; }
  const auto & counts = object(*counts_item, "counts");
  CTestCounts actual;
  if (!ctest_counts(path, &actual, why)) return false;
  const auto number = [&](const std::string & key, int expected) {
    const auto * item = find(counts, key);
    return item && std::holds_alternative<double>(*item) && std::get<double>(*item) == expected;
  };
  if (!number("PASS", actual.passed) || !number("FAIL", actual.failed) ||
      !number("NOT_RUN", actual.skipped) || !number("UNAVAILABLE", 0) || !number("NOT_VERIFIED", 0)) {
    *why = "report counts disagree with CTest XML (PASS/FAIL/NOT_RUN and non-CTest statuses)";
    return false;
  }
  const auto & cases = array(*cases_item, "cases");
  if (static_cast<int>(cases.size()) != actual.total) { *why = "report case total disagrees with CTest XML"; return false; }
  std::set<std::string> report_names;
  for (const auto & item : cases) {
    const auto & entry = object(item, "case");
    const auto * name_item = find(entry, "name");
    if (!name_item) name_item = find(entry, "id");
    if (!name_item || !std::holds_alternative<std::string>(*name_item) ||
      std::get<std::string>(*name_item).empty())
    {
      *why = "CTest-backed case is missing a non-empty name";
      return false;
    }
    const auto case_name = std::get<std::string>(*name_item);
    if (!report_names.insert(case_name).second) {
      *why = "report contains duplicate CTest case name: " + case_name;
      return false;
    }
    const auto * status_item = find(entry, "status");
    if (!status_item || !std::holds_alternative<std::string>(*status_item)) {
      *why = "CTest-backed case is missing a string status";
      return false;
    }
    try {
      const auto case_status = status(std::get<std::string>(*status_item));
      const auto canonical = std::string(name(case_status));
      const auto record = std::find_if(actual.records.begin(), actual.records.end(),
        [&](const CTestRecord & candidate) { return candidate.name == case_name; });
      if (record == actual.records.end()) { *why = "report case name not found in CTest XML: " + case_name; return false; }
      if (record->status != canonical) {
        *why = "report case status disagrees with CTest XML for " + case_name;
        return false;
      }
    } catch (const std::exception &) {
      *why = "CTest-backed case has unknown status";
      return false;
    }
  }
  if (report_names.size() != actual.records.size()) {
    *why = "report case names disagree with CTest XML";
    return false;
  }
  if (const auto * status_item = find(root, "status"); status_item &&
    std::holds_alternative<std::string>(*status_item) &&
    lower_token(std::get<std::string>(*status_item)) == "warn")
  {
    return true;
  }
  return true;
}

bool negative_claim_rejection_valid(const Object & root, const Object & declaration,
                                    std::string * why)
{
  bool negative_present = false;
  const bool negative = json_bool(declaration, "negative_test", &negative_present);
  bool expected_present = false;
  const bool expected_failure = json_bool(declaration, "expected_failure", &expected_present);
  if (!negative_present && !expected_present) return true;
  if (!negative_present || !expected_present || !negative || !expected_failure) {
    *why = "negative evidence requires negative_test=true and expected_failure=true";
    return false;
  }
  if (!json_bool(root, "production_claim_rejected")) {
    *why = "negative evidence did not prove production claim rejection";
    return false;
  }
  std::vector<std::string> status_reasons;
  const auto status_value = status_hint(root, &status_reasons, nullptr);
  // Do not allow a PASS alias to coexist with the expected rejection.
  if (!status_value || !status_reasons.empty() || *status_value != Status::Fail) {
    *why = "negative evidence must report FAIL for the rejected production claim";
    return false;
  }
  const auto expected_code_item = find(declaration, "expected_diagnostic_code");
  if (!expected_code_item || !std::holds_alternative<std::string>(*expected_code_item) ||
    trim_token(std::get<std::string>(*expected_code_item)).empty())
  {
    *why = "negative evidence requires expected_diagnostic_code";
    return false;
  }
  const auto diagnostic_code = lower_token(trim_token(std::get<std::string>(*expected_code_item)));
  if (const auto * alias = find(declaration, "diagnostic_code")) {
    if (!std::holds_alternative<std::string>(*alias) ||
      lower_token(trim_token(std::get<std::string>(*alias))) != diagnostic_code)
    {
      *why = "negative diagnostic code aliases disagree";
      return false;
    }
  }
  if (diagnostic_code != "calibration_promotion") {
    *why = "negative evidence diagnostic code is not the fixed calibration_promotion code";
    return false;
  }
  const Json * diagnostics_item = find(root, "diagnostics");
  if (!diagnostics_item) diagnostics_item = find(root, "bundle_diagnostics");
  if (!diagnostics_item || !std::holds_alternative<Object>(*diagnostics_item)) {
    *why = "negative evidence diagnostics are missing";
    return false;
  }
  const auto & diagnostics = std::get<Object>(*diagnostics_item);
  const auto * warnings_item = find(diagnostics, "warnings");
  if (!warnings_item || !std::holds_alternative<Array>(*warnings_item) ||
    !std::get<Array>(*warnings_item).empty())
  {
    *why = "negative evidence diagnostics.warnings must be an empty array";
    return false;
  }
  const auto * errors_item = find(diagnostics, "errors");
  if (!errors_item || !std::holds_alternative<Array>(*errors_item) || std::get<Array>(*errors_item).size() != 1U) {
    *why = "negative evidence must contain exactly one diagnostic error";
    return false;
  }
  const auto & error_value = std::get<Array>(*errors_item).front();
  if (!std::holds_alternative<Object>(error_value)) { *why = "negative diagnostic entry is malformed"; return false; }
  const auto * code_item = find(std::get<Object>(error_value), "code");
  if (!code_item || !std::holds_alternative<std::string>(*code_item) ||
    lower_token(trim_token(std::get<std::string>(*code_item))) != diagnostic_code)
  {
    *why = "negative diagnostic code does not match calibration_promotion";
    return false;
  }
  const auto integer_nonzero = [](const std::optional<double> & value) {
    return value && std::isfinite(*value) && *value > 0.0 &&
      std::floor(*value) == *value;
  };
  const auto expected_exit = json_number(declaration, "expected_exit_code");
  const auto expected_exit_alias = json_number(declaration, "expected_failure_exit_code");
  if (expected_exit && expected_exit_alias && *expected_exit != *expected_exit_alias) {
    *why = "negative evidence expected exit-code aliases disagree";
    return false;
  }
  const auto expected = expected_exit ? expected_exit : expected_exit_alias;
  std::optional<double> observed;
  for (const auto * key : {"observed_exit_code", "exit_code", "returncode", "process_exit_code"}) {
    const auto candidate = json_number(root, key);
    if (!candidate) continue;
    if (observed && *observed != *candidate) {
      *why = "negative evidence observed exit-code aliases disagree";
      return false;
    }
    observed = candidate;
  }
  if (!integer_nonzero(expected) || !integer_nonzero(observed) || *expected != *observed) {
    *why = "negative evidence expected/observed exit codes must match and be non-zero";
    return false;
  }
  // ``fixture_sha256`` identifies the deliberately failing fixture payload,
  // not the bytes of the enclosing report JSON.  Accept the historical
  // aliases on each side, but require every supplied alias to be valid and
  // mutually consistent before comparing declaration to report.
  const auto collect_hash_aliases = [&](const Object & object_value,
                                        const std::initializer_list<const char *> keys,
                                        const std::string & context,
                                        std::vector<std::string> * values) {
    for (const auto * key : keys) {
      const auto * item = find(object_value, key);
      if (!item) continue;
      if (!std::holds_alternative<std::string>(*item) ||
        !sha256_text(std::get<std::string>(*item)))
      {
        *why = context + " " + key + " sha256 must be 64 hexadecimal characters";
        return false;
      }
      values->push_back(lower_token(std::get<std::string>(*item)));
    }
    return true;
  };
  std::vector<std::string> expected_fixtures;
  if (!collect_hash_aliases(declaration, {"fixture_sha256", "expected_fixture_sha256"},
      "negative evidence declaration", &expected_fixtures)) return false;
  if (expected_fixtures.empty()) {
    *why = "negative evidence requires fixture_sha256";
    return false;
  }
  if (std::any_of(expected_fixtures.begin() + 1U, expected_fixtures.end(),
      [&](const std::string & value) { return value != expected_fixtures.front(); }))
  {
    *why = "negative evidence fixture hash aliases disagree";
    return false;
  }

  std::vector<std::string> actual_fixtures;
  if (!collect_hash_aliases(root,
      {"fixture_sha256", "fixture_hash", "expected_fixture_sha256"},
      "negative evidence source", &actual_fixtures)) return false;
  if (const auto * fixture_item = find(root, "fixture")) {
    if (!std::holds_alternative<Object>(*fixture_item)) {
      *why = "negative evidence fixture must be an object";
      return false;
    }
    if (!collect_hash_aliases(std::get<Object>(*fixture_item), {"sha256", "sha"},
        "negative evidence fixture", &actual_fixtures)) return false;
  }
  if (std::any_of(actual_fixtures.begin() + (actual_fixtures.empty() ? 0U : 1U),
      actual_fixtures.end(), [&](const std::string & value) {
        return value != actual_fixtures.front();
      }))
  {
    *why = "negative evidence fixture hash aliases disagree";
    return false;
  }
  if (actual_fixtures.empty() || actual_fixtures.front() != expected_fixtures.front()) {
    *why = "negative evidence fixture SHA-256 does not match declaration";
    return false;
  }
  return true;
}

struct Source { std::string id; std::string kind; fs::path path; std::string digest; Status result{Status::Fail}; std::vector<std::string> reasons; };
Object source_json(const Source & source)
{ Array reasons; for (const auto & item : source.reasons) reasons.emplace_back(item); return Object{{"id",source.id},{"kind",source.kind},{"path",source.path.generic_string()},{"sha256",source.digest},{"status",std::string(name(source.result))},{"reasons",reasons}}; }
struct Artifact { std::string role; fs::path path; std::string digest; Status result{Status::Fail}; std::vector<std::string> reasons; };
Object artifact_json(const Artifact & artifact)
{ Array reasons; for (const auto & item : artifact.reasons) reasons.emplace_back(item); return Object{{"role",artifact.role},{"path",artifact.path.generic_string()},{"sha256",artifact.digest},{"status",std::string(name(artifact.result))},{"reasons",reasons}}; }

}  // namespace

int run(const fs::path & config_path, const fs::path & output_dir, std::ostream & output, std::ostream & error)
{
  const auto config = object(Parser(read_file(config_path)).parse(), "audit configuration");
  if (const auto * version = find(config, "schema_version"); !version || !std::holds_alternative<double>(*version) || std::get<double>(*version) != 1.0) throw std::runtime_error("audit configuration requires schema_version 1");
  const auto & candidate = object(required(config, "candidate", "audit configuration"), "candidate");
  const auto head = text(required(candidate, "head", "candidate"), "candidate.head"); const auto baseline = text(required(candidate, "main_baseline", "candidate"), "candidate.main_baseline");
  if (!git_sha(head) || !git_sha(baseline)) throw std::runtime_error("candidate Git SHA must be 40 hexadecimal characters");
  for (const auto * key : {"head_sha", "candidate_sha", "candidate_commit", "observed_head", "observed_head_sha"}) {
    if (const auto * value = find(candidate, key)) {
      if (!std::holds_alternative<std::string>(*value) || !git_sha(std::get<std::string>(*value)) || lower_token(std::get<std::string>(*value)) != lower_token(head)) {
        throw std::runtime_error(std::string("candidate SHA alias disagrees: ") + key);
      }
    }
  }
  for (const auto * key : {"main_baseline_sha", "origin_main_sha", "baseline_main", "baseline_sha", "observed_main_baseline", "observed_origin_main"}) {
    if (const auto * value = find(candidate, key)) {
      if (!std::holds_alternative<std::string>(*value) || !git_sha(std::get<std::string>(*value)) || lower_token(std::get<std::string>(*value)) != lower_token(baseline)) {
        throw std::runtime_error(std::string("candidate baseline SHA alias disagrees: ") + key);
      }
    }
  }
  const auto branch = optional_text(candidate, "branch");
  if (!branch || trim_token(*branch).empty()) throw std::runtime_error("candidate.branch must be a non-empty string");
  if (!boolean(required(candidate, "worktree_clean", "candidate"), "candidate.worktree_clean")) throw std::runtime_error("candidate worktree_clean=false is fail-closed");
  if (fs::exists(output_dir)) throw std::runtime_error("output directory already exists; refusing to overwrite: " + output_dir.string());
  if (has_symlink_component(output_dir.parent_path())) throw std::runtime_error("output directory parent contains a symlink");
  const auto & declarations = array(required(config, "sources", "audit configuration"), "sources"); if (declarations.empty()) throw std::runtime_error("sources must not be empty");
  std::vector<Source> sources; std::set<std::string> ids;
  for (const auto & declaration : declarations) {
    const auto & source_config = object(declaration, "source"); Source source;
    source.id = text(required(source_config, "id", "source"), "source.id");
    const auto raw_kind = text(required(source_config, "kind", "source"), "source.kind");
    source.kind = canonical_kind(raw_kind);
    source.path = text(required(source_config, "path", "source"), "source.path");
    if (!ids.insert(source.id).second) throw std::runtime_error("duplicate source id: " + source.id);
    if (source.kind.empty() || allowed_kinds().count(source.kind) == 0U) {
      source.result = Status::Fail;
      source.reasons.push_back("source kind is unknown or not allow-listed");
      sources.push_back(std::move(source));
      continue;
    }
    bool skip_ctest_invalid = false;
    if (const auto * skip = find(source_config, "skip_ctest")) {
      if (!std::holds_alternative<bool>(*skip) || std::get<bool>(*skip)) {
        skip_ctest_invalid = true;
        source.result = Status::Fail;
        source.reasons.push_back("skip_ctest is not permitted in explicit admission mode");
      }
    }
    const auto absence = absence_status(source_config, &source.reasons, "source");
    if (absence && source_config.find("absence_status") != source_config.end() &&
      std::holds_alternative<std::string>(*find(source_config, "absence_status")) &&
      lower_token(std::get<std::string>(*find(source_config, "absence_status"))) == "warn")
    {
      source.result = Status::Fail;
      source.reasons.push_back("absence_status WARN is not permitted");
    }
    std::uintmax_t size = 0U;
    const bool regular = regular_nonsymlink(source.path, &size);
    if (!regular || size == 0U) { if (!absence || skip_ctest_invalid) { source.result = Status::Fail; source.reasons.push_back(!regular ? "source is not a regular non-link file" : "missing or empty required source"); } else if (!regular && (fs::exists(source.path) || has_symlink_component(source.path))) { source.result = Status::Fail; source.reasons.push_back("source path is unsafe or not a regular non-link file"); } else { source.result = *absence; source.reasons.push_back("source unavailable: explicit " + std::string(name(*absence))); } sources.push_back(std::move(source)); continue; }
    bool negative_valid = false;
    try {
      source.digest = sha256(source.path);
      const auto expected = optional_text(source_config, "sha256");
      if (!expected) source.reasons.push_back("source.sha256 is required for present input");
      else if (!sha256_text(*expected) || lower_token(source.digest) != lower_token(*expected)) source.reasons.push_back("declared SHA-256 mismatch");
      if (absence) {
        source.reasons.push_back("absence_status contradicts present source");
      }
      const auto root = object(Parser(read_file(source.path)).parse(), "source JSON");
      const auto schema = find(root, "schema_version"); if (!schema || !std::holds_alternative<double>(*schema) || std::get<double>(*schema) != 1.0) source.reasons.push_back("unknown or missing schema_version");
      bool negative_declared = false;
      (void)json_bool(source_config, "negative_test", &negative_declared);
      bool expected_failure_declared = false;
      (void)json_bool(source_config, "expected_failure", &expected_failure_declared);
      if (const auto * skip = find(root, "skip_ctest")) {
        if (!std::holds_alternative<bool>(*skip) || std::get<bool>(*skip)) {
          source.reasons.push_back("skip_ctest is not permitted in explicit admission mode");
        }
      }
      std::string raw_status;
      std::optional<Status> reported_status = status_hint(root, &source.reasons, &raw_status);
      const bool reported_warn = lower_token(raw_status) == "warn";
      if (reported_status && *reported_status == Status::Fail) source.reasons.push_back("source reports " + raw_status);
      else if (reported_status && *reported_status != Status::Pass) {
        source.result = *reported_status;
        source.reasons.push_back("source reports " + raw_status + "; cannot count as release PASS");
      }
      if (unsafe_claim(Json(root))) source.reasons.push_back("production or hardware claim detected");
      std::string reason; if (!safe_fields(root, &reason)) source.reasons.push_back(reason);
      if (is_hardware_kind(source.kind) && reported_status && *reported_status == Status::Pass) {
        source.reasons.push_back("hardware source cannot claim PASS");
      }
      if (!counts_valid(root, &reason)) source.reasons.push_back(reason);
      if (reported_status && !reported_warn && !status_matches_results(root, *reported_status, &reason)) source.reasons.push_back(reason);
      if (reported_warn && !warn_matches_results(root, &reason)) source.reasons.push_back(reason);
      if (const auto ctest_path = optional_text(source_config, "ctest_xml")) {
        if (!ctest_matches_report(root, *ctest_path, &reason)) source.reasons.push_back(reason);
      } else if (find(root, "counts") || find(root, "cases")) {
        source.reasons.push_back("CTest-backed report requires explicit ctest_xml");
      }
      std::optional<std::string> declared_commit;
      std::optional<std::string> declared_base;
      for (const auto * key : {"head", "commit", "candidate_commit", "head_sha", "candidate_sha"}) {
        if (const auto * value = find(root, key)) {
          if (!std::holds_alternative<std::string>(*value)) source.reasons.push_back(std::string(key) + " must be a string");
          else if (!declared_commit) declared_commit = std::get<std::string>(*value);
          else if (lower_token(*declared_commit) != lower_token(std::get<std::string>(*value))) source.reasons.push_back("candidate Git SHA aliases disagree");
        }
      }
      for (const auto * key : {"main_baseline", "main_baseline_sha", "origin_main_sha", "baseline_main", "baseline_sha"}) {
        if (const auto * value = find(root, key)) {
          if (!std::holds_alternative<std::string>(*value)) source.reasons.push_back(std::string(key) + " must be a string");
          else if (!declared_base) declared_base = std::get<std::string>(*value);
          else if (lower_token(*declared_base) != lower_token(std::get<std::string>(*value))) source.reasons.push_back("main baseline SHA aliases disagree");
        }
      }
      if (declared_commit && lower_token(*declared_commit) != lower_token(head)) source.reasons.push_back("candidate Git SHA mismatch");
      if (declared_base && lower_token(*declared_base) != lower_token(baseline)) source.reasons.push_back("main baseline SHA mismatch");
      const auto configured_ctest_path = optional_text(source_config, "ctest_xml");
      const auto declared_xml_digest = optional_text(source_config, "ctest_xml_sha256");
      const auto root_xml_digest = find(root, "ctest_xml_sha256");
      if (configured_ctest_path) {
        // The XML digest is an admission-side binding.  A digest copied into
        // the report itself is not authoritative and must not satisfy this
        // requirement when the explicit configuration field is absent.
        if (!declared_xml_digest) {
          source.reasons.push_back("ctest_xml_sha256 is required when ctest_xml is declared");
        } else if (!sha256_text(*declared_xml_digest)) {
          source.reasons.push_back("ctest_xml_sha256 must be a 64-character hexadecimal SHA-256");
        } else {
          try {
            if (sha256(*configured_ctest_path) != lower_token(*declared_xml_digest)) {
              source.reasons.push_back("ctest_xml_sha256 does not match CTest XML bytes");
            }
          } catch (const std::exception & exception) { source.reasons.push_back(exception.what()); }
        }
        if (root_xml_digest) {
          if (!std::holds_alternative<std::string>(*root_xml_digest) ||
            !sha256_text(std::get<std::string>(*root_xml_digest)))
          {
            source.reasons.push_back("source ctest_xml_sha256 must be a 64-character hexadecimal SHA-256");
          } else if (declared_xml_digest &&
            lower_token(std::get<std::string>(*root_xml_digest)) != lower_token(*declared_xml_digest))
          {
            source.reasons.push_back("source ctest_xml_sha256 disagrees with configured digest");
          }
        }
      } else if (declared_xml_digest || root_xml_digest) {
        source.reasons.push_back("ctest_xml_sha256 requires explicit ctest_xml");
      }
      std::string negative_reason;
      bool negative_unsafe_claim_only_expected = false;
      if (negative_declared || expected_failure_declared) {
        if (!negative_claim_rejection_valid(root, source_config, &negative_reason)) {
          source.reasons.push_back(negative_reason);
        } else {
          // The deliberately failing fixture is a PASS for this negative test
          // only after every rejection assertion above has matched exactly.
          negative_valid = true;
          source.result = Status::Pass;
          // The initial recursive scan deliberately remains strict.  Only
          // after the complete negative contract succeeds may the exact
          // top-level production_ready=true claim be treated as expected.
          negative_unsafe_claim_only_expected = !unsafe_claim(Json(root), true);
          if (negative_unsafe_claim_only_expected) {
            source.reasons.erase(std::remove(source.reasons.begin(), source.reasons.end(),
              "production or hardware claim detected"), source.reasons.end());
          }
        }
      }
      if (source.kind == "scenario_benchmark" || source.kind == "scenario" ||
        source.kind == "evidence" || source.kind == "evidence_report" ||
        source.kind == "evidence_bundle" || source.kind == "bundle" ||
        source.kind == "calibration" || source.kind == "qualification" ||
        source.kind == "model_qualification") {
        if (!expected_synthetic(root, &reason, negative_valid && negative_unsafe_claim_only_expected)) {
          source.reasons.push_back(reason);
        }
      }
      if (source.kind == "ros_e2e" || source.kind == "ros_message_e2e") {
        const auto * live = find(root, "node_liveness");
        if (!live || !std::holds_alternative<Object>(*live)) {
          source.result = Status::NotVerified;
          source.reasons.push_back("node_liveness evidence missing; E2E cannot count as PASS");
        } else {
          const auto & evidence = std::get<Object>(*live);
          const auto bool_true = [&](const char * key) {
            const auto * value = find(evidence, key);
            return value && std::holds_alternative<bool>(*value) && std::get<bool>(*value);
          };
          const auto number_zero = [&](const char * key) {
            const auto * value = find(evidence, key);
            return value && std::holds_alternative<double>(*value) && std::get<double>(*value) == 0.0;
          };
          const auto * matches = find(evidence, "exit_code_matches");
          if (!bool_true("alive_before_sampling") || !bool_true("alive_during_sampling") ||
            !bool_true("alive_after_sampling") || !number_zero("expected_exit_code") ||
            !number_zero("observed_exit_code") || !matches || !std::holds_alternative<bool>(*matches) ||
            !std::get<bool>(*matches))
          {
            source.result = Status::NotVerified;
            source.reasons.push_back("node_liveness evidence contradicts sampling or expected exit");
          }
          const auto * cases_value = find(root, "cases");
          if (!cases_value || !std::holds_alternative<Array>(*cases_value)) {
            source.result = Status::NotVerified;
            source.reasons.push_back("node_liveness case evidence is missing");
          } else {
            for (const auto & case_value : std::get<Array>(*cases_value)) {
              if (!std::holds_alternative<Object>(case_value)) { source.result = Status::NotVerified; source.reasons.push_back("node_liveness case is not an object"); break; }
              const auto & case_object = std::get<Object>(case_value);
              const auto * case_status = find(case_object, "status");
              const auto * case_live = find(case_object, "node_liveness");
              if (!case_status || !std::holds_alternative<std::string>(*case_status)) { source.result = Status::NotVerified; source.reasons.push_back("node_liveness case status is missing"); break; }
              const auto case_status_value = lower_token(std::get<std::string>(*case_status));
              if (case_status_value != "pass") continue; // non-PASS cases are covered by aggregate status.
              bool applicable = true;
              if (const auto * marker = find(case_object, "node_liveness_applicable")) {
                if (!std::holds_alternative<bool>(*marker)) { source.result = Status::NotVerified; source.reasons.push_back("node_liveness_applicable must be boolean"); break; }
                applicable = std::get<bool>(*marker);
              }
              if (!applicable) {
                // Non-node cases may explicitly opt out, but the report must
                // still carry the structured sentinel object emitted by the
                // E2E reporter.  A bare false marker must not become a way to
                // omit liveness evidence altogether.
                if (!case_live || !std::holds_alternative<Object>(*case_live)) {
                  source.result = Status::NotVerified;
                  source.reasons.push_back("PASS node_liveness case is missing");
                  break;
                }
                const auto & case_evidence = std::get<Object>(*case_live);
                const auto case_bool_false = [&](const char * key) {
                  const auto * value = find(case_evidence, key);
                  return value && std::holds_alternative<bool>(*value) && !std::get<bool>(*value);
                };
                const auto case_number = [&](const char * key, double expected) {
                  const auto * value = find(case_evidence, key);
                  return value && std::holds_alternative<double>(*value) &&
                    std::isfinite(std::get<double>(*value)) && std::get<double>(*value) == expected;
                };
                const auto * case_matches = find(case_evidence, "exit_code_matches");
                if (!case_bool_false("alive_before_sampling") ||
                  !case_bool_false("alive_during_sampling") ||
                  !case_bool_false("alive_after_sampling") ||
                  !case_number("expected_exit_code", -1.0) ||
                  !case_number("observed_exit_code", -1.0) ||
                  !case_matches || !std::holds_alternative<bool>(*case_matches) ||
                  std::get<bool>(*case_matches))
                {
                  source.result = Status::NotVerified;
                  source.reasons.push_back("PASS node_liveness case sentinel is incomplete or invalid");
                  break;
                }
                continue; // lifecycle/expected-failure/unavailable cases.
              }
              if (!case_live || !std::holds_alternative<Object>(*case_live)) { source.result = Status::NotVerified; source.reasons.push_back("PASS node_liveness case is missing"); break; }
              const auto & case_evidence = std::get<Object>(*case_live);
              const auto case_bool_true = [&](const char * key) { const auto * value = find(case_evidence, key); return value && std::holds_alternative<bool>(*value) && std::get<bool>(*value); };
              const auto case_zero = [&](const char * key) { const auto * value = find(case_evidence, key); return value && std::holds_alternative<double>(*value) && std::get<double>(*value) == 0.0; };
              const auto * case_matches = find(case_evidence, "exit_code_matches");
              if (!case_bool_true("alive_before_sampling") || !case_bool_true("alive_during_sampling") || !case_bool_true("alive_after_sampling") || !case_zero("expected_exit_code") || !case_zero("observed_exit_code") || !case_matches || !std::holds_alternative<bool>(*case_matches) || !std::get<bool>(*case_matches)) {
                source.result = Status::NotVerified; source.reasons.push_back("PASS node_liveness case is incomplete or invalid"); break;
              }
            }
          }
        }
      }
      const auto non_fatal_status_reason = [](const std::string & item) {
        return item == "node_liveness evidence missing; E2E cannot count as PASS" ||
               item == "node_liveness evidence contradicts sampling or expected exit" ||
               item.find("node_liveness case") == 0U ||
               item.find("PASS node_liveness case") == 0U ||
               item == "source reports WARN; cannot count as release PASS" ||
               item.find("source reports UNAVAILABLE; cannot count as release PASS") == 0U ||
               item.find("source reports NOT_RUN; cannot count as release PASS") == 0U ||
               item.find("source reports NOT_VERIFIED; cannot count as release PASS") == 0U;
      };
      if (negative_valid) {
        // All reasons related to the expected FAIL are intentionally
        // informational; any additional reason remains fatal below.
        const auto expected_reason = [&](const std::string & item) {
          return item == "source reports FAIL" ||
            (item == "production or hardware claim detected" && negative_unsafe_claim_only_expected);
        };
        if (std::any_of(source.reasons.begin(), source.reasons.end(),
          [&](const std::string & item) { return !expected_reason(item); })) source.result = Status::Fail;
        else source.result = Status::Pass;
      } else if (is_absence_status(source.result)) {
        if (std::any_of(source.reasons.begin(), source.reasons.end(), [&](const std::string & item) { return !non_fatal_status_reason(item); })) source.result = Status::Fail;
      } else {
        source.result = source.reasons.empty() ? Status::Pass : Status::Fail;
      }
    } catch (const std::exception & exception) { source.result = Status::Fail; source.reasons.push_back(std::string("unreadable or malformed source: ") + exception.what()); }
    sources.push_back(std::move(source));
  }
  std::vector<Artifact> artifacts; std::set<std::string> roles;
  if (const auto * declarations_artifacts = find(config, "artifacts")) {
    for (const auto & declaration : array(*declarations_artifacts, "artifacts")) {
      const auto & item = object(declaration, "artifact"); Artifact artifact;
      artifact.role = text(required(item, "role", "artifact"), "artifact.role"); artifact.path = text(required(item, "path", "artifact"), "artifact.path");
      if (!roles.insert(artifact.role).second) throw std::runtime_error("duplicate artifact role: " + artifact.role);
      const auto absence = absence_status(item, &artifact.reasons, "artifact");
      std::uintmax_t size = 0U;
      const bool regular = regular_nonsymlink(artifact.path, &size);
      if (!regular || size == 0U) {
        if (regular && size == 0U) {
          artifact.result = absence ? *absence : Status::Fail;
          artifact.reasons.push_back(
            absence ? "artifact unavailable: explicit " + std::string(name(*absence)) :
            "missing or empty required artifact");
        } else if (absence && !fs::exists(artifact.path) && !has_symlink_component(artifact.path)) {
          artifact.result = *absence;
          artifact.reasons.push_back("artifact unavailable: explicit " + std::string(name(*absence)));
        } else {
          artifact.result = Status::Fail;
          artifact.reasons.push_back("artifact is not a regular non-link file");
        }
      } else {
        artifact.digest = sha256(artifact.path); const auto expected = optional_text(item, "sha256");
        if (!expected) artifact.reasons.push_back("artifact.sha256 is required for present input");
        else if (!sha256_text(*expected) || lower_token(artifact.digest) != lower_token(*expected)) artifact.reasons.push_back("declared SHA-256 mismatch");
        if (absence) artifact.reasons.push_back("absence_status contradicts present artifact");
        artifact.result = artifact.reasons.empty() ? Status::Pass : Status::Fail;
      }
      artifacts.push_back(std::move(artifact));
    }
  }
  std::sort(sources.begin(), sources.end(), [](const Source & left, const Source & right) {
    return left.id < right.id;
  });
  std::sort(artifacts.begin(), artifacts.end(), [](const Artifact & left, const Artifact & right) {
    return left.role < right.role;
  });
  Status overall = Status::Pass; for (const auto & source : sources) { if (source.result == Status::Fail) overall = Status::Fail; else if (overall == Status::Pass && source.result != Status::Pass) overall = Status::NotVerified; } for (const auto & artifact : artifacts) { if (artifact.result == Status::Fail) overall = Status::Fail; else if (overall == Status::Pass && artifact.result != Status::Pass) overall = Status::NotVerified; }
  Array source_values; for (const auto & source : sources) source_values.emplace_back(source_json(source));
  Array artifact_values; for (const auto & artifact : artifacts) artifact_values.emplace_back(artifact_json(artifact));
  Object manifest{{"schema_version",1.0},{"manifest_type","software_candidate_release_manifest"},{"canonicalization","sorted JSON object keys and source/artifact IDs; no generated timestamp"},{"audit_tool",Object{{"name","release_manifest_audit"},{"version","1"}}},{"status",std::string(name(overall))},{"candidate",candidate},{"sources",source_values},{"artifacts",artifact_values},{"safety_boundary",Object{{"serial_enabled",false},{"dry_run",true},{"allow_fire",false},{"fire_command",0.0},{"yaw_vel",0.0},{"pitch_vel",0.0},{"yaw_acc",0.0},{"pitch_acc",0.0}}},{"limitations",Array{"Manifest validation does not validate a real camera, Orin, CDC serial, robot, gimbal, firing, formal model/calibration, latency, hit rate, or competition performance."}}};
  const auto canonical = render(Json(manifest));
  fs::create_directories(output_dir); write_new(output_dir / "release-manifest.json", canonical + "\n");
  std::ostringstream markdown; markdown << "# Software candidate release manifest\n\n- Status: `" << name(overall) << "`\n- Candidate HEAD: `" << head << "`\n- Main baseline: `" << baseline << "`\n- Canonical JSON: `release-manifest.json`\n\n| Source | Kind | Status | SHA-256 | Reasons |\n|---|---|---|---|---|\n"; for (const auto & source : sources) { markdown << "| `" << source.id << "` | `" << source.kind << "` | `" << name(source.result) << "` | `" << source.digest << "` | "; for (std::size_t i=0;i<source.reasons.size();++i) { if(i) markdown << "; "; markdown << source.reasons[i]; } markdown << " |\n"; } markdown << "\n| Artifact role | Status | SHA-256 | Reasons |\n|---|---|---|---|\n"; for (const auto & artifact : artifacts) { markdown << "| `" << artifact.role << "` | `" << name(artifact.result) << "` | `" << artifact.digest << "` | "; for (std::size_t i=0;i<artifact.reasons.size();++i) { if(i) markdown << "; "; markdown << artifact.reasons[i]; } markdown << " |\n"; } markdown << "\nManifest validation passing does not mean real camera, Orin, CDC, gimbal, firing, or competition-performance validation passed.\n"; write_new(output_dir / "SUMMARY.md", markdown.str());
  const auto manifest_hash = sha256(output_dir / "release-manifest.json"); const auto markdown_hash = sha256(output_dir / "SUMMARY.md"); std::ostringstream sums; for (const auto & source : sources) if (!source.digest.empty()) sums << source.digest << "  source/" << source.id << "\n"; for (const auto & artifact : artifacts) if (!artifact.digest.empty()) sums << artifact.digest << "  artifact/" << artifact.role << "\n"; sums << manifest_hash << "  release-manifest.json\n" << markdown_hash << "  SUMMARY.md\n"; write_new(output_dir / "SHA256SUMS", sums.str());
  output << "status=" << name(overall) << "\nmanifest=" << (output_dir / "release-manifest.json") << "\n"; if (overall == Status::Fail) error << "release manifest audit failed closed\n"; return overall == Status::Fail ? 1 : (overall == Status::Pass ? 0 : 2);
}

}  // namespace release_manifest_audit
