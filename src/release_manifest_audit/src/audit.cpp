#include "release_manifest_audit/audit.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <fcntl.h>
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
bool unsafe_claim(const Json & value)
{
  if (std::holds_alternative<Object>(value)) { for (const auto & [key, child] : std::get<Object>(value)) { const auto lower = [&] { std::string result; for (char c : key) result += static_cast<char>(std::tolower(static_cast<unsigned char>(c))); return result; }(); if ((lower == "production_ready" || lower == "hardware_validation" || lower == "firing_validated" || lower == "gimbal_closed_loop_validated" || lower == "real_hit_rate_computed") && std::holds_alternative<bool>(child) && std::get<bool>(child)) return true; if (unsafe_claim(child)) return true; } }
  if (std::holds_alternative<Array>(value)) for (const auto & child : std::get<Array>(value)) if (unsafe_claim(child)) return true;
  return false;
}
bool safe_fields(const Object & root, std::string * why)
{
  const Object * safety = &root; if (const auto * nested = find(root, "safety_assertions")) safety = &object(*nested, "safety_assertions"); else if (const auto * nested = find(root, "safety")) safety = &object(*nested, "safety"); else if (const auto * nested = find(root, "safety_defaults")) safety = &object(*nested, "safety_defaults");
  const std::vector<std::pair<std::string, bool>> values{{"serial_enabled", false}, {"dry_run", true}, {"allow_fire", false}};
  for (const auto & [key, expected] : values) { const auto * item = find(*safety, key); if (!item || !std::holds_alternative<bool>(*item) || std::get<bool>(*item) != expected) { *why = "unsafe or missing " + key; return false; } }
  for (const auto & key : {"fire_command", "yaw_vel", "pitch_vel", "yaw_acc", "pitch_acc"}) { const auto * item = find(*safety, key); if (!item || !std::holds_alternative<double>(*item) || std::get<double>(*item) != 0.0) { *why = "unsafe or missing " + std::string(key); return false; } }
  return true;
}
bool expected_synthetic(const Object & root, std::string * why)
{
  for (const auto & [key, expected] : std::vector<std::pair<std::string, bool>>{{"synthetic", true}, {"test_only", true}, {"production_ready", false}}) { const auto * item = find(root, key); if (!item || !std::holds_alternative<bool>(*item) || std::get<bool>(*item) != expected) { *why = "missing or contradictory " + key; return false; } } return true;
}
bool counts_valid(const Object & root, std::string * why)
{
  const auto * counts_item = find(root, "counts"); const auto * cases_item = find(root, "cases"); if (!counts_item || !cases_item) return true;
  const auto & counts = object(*counts_item, "counts"); const auto & cases = array(*cases_item, "cases"); std::map<std::string, int> actual{{"PASS",0},{"FAIL",0},{"UNAVAILABLE",0},{"NOT_RUN",0},{"NOT_VERIFIED",0}};
  for (const auto & item : cases) { const auto & entry = object(item, "case"); const auto value = text(required(entry, "status", "case"), "case.status"); if (!actual.count(value)) { *why = "case has unknown status"; return false; } ++actual[value]; }
  for (const auto & [key, total] : actual) { const auto * item = find(counts, key); if (!item || !std::holds_alternative<double>(*item) || std::get<double>(*item) != total) { *why = "counts disagree with cases for " + key; return false; } } return true;
}
struct CTestCounts { int total{0}; int passed{0}; int failed{0}; int skipped{0}; };
bool ctest_counts(const fs::path & path, CTestCounts * result, std::string * why)
{
  std::string xml;
  try { xml = read_file(path); } catch (const std::exception & exception) { *why = std::string("CTest XML cannot be read: ") + exception.what(); return false; }
  if (xml.empty()) { *why = "CTest XML is empty"; return false; }
  const std::regex test_open(R"(<\s*Test\b([^>]*)>)", std::regex::icase);
  const std::regex test_close(R"(<\s*/\s*Test\s*>)", std::regex::icase);
  const std::regex status_attribute(R"(\bStatus\s*=\s*([\"'])([^\"']*)\1)", std::regex::icase);
  const auto begin = std::sregex_iterator(xml.begin(), xml.end(), test_open);
  const auto end = std::sregex_iterator();
  const auto openings = static_cast<int>(std::distance(begin, end));
  const auto closings = static_cast<int>(std::distance(std::sregex_iterator(xml.begin(), xml.end(), test_close), end));
  if (openings == 0 || openings != closings) { *why = "CTest XML is malformed or contains no complete Test records"; return false; }
  for (auto it = begin; it != end; ++it) {
    std::smatch match;
    const auto attributes = (*it)[1].str();
    if (!std::regex_search(attributes, match, status_attribute)) { *why = "CTest Test record has no Status attribute"; return false; }
    std::string value = match[2].str();
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    ++result->total;
    if (value == "passed") ++result->passed;
    else if (value == "failed") ++result->failed;
    else if (value == "notrun" || value == "not_run" || value == "skipped") ++result->skipped;
    else { *why = "CTest Test record has unsupported Status: " + value; return false; }
  }
  return true;
}
bool ctest_matches_report(const Object & root, const fs::path & path, std::string * why)
{
  const auto * counts_item = find(root, "counts"); const auto * cases_item = find(root, "cases");
  if (!counts_item && !cases_item) return true;
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
  if (!boolean(required(candidate, "worktree_clean", "candidate"), "candidate.worktree_clean")) throw std::runtime_error("candidate worktree_clean=false is fail-closed");
  if (fs::exists(output_dir)) throw std::runtime_error("output directory already exists; refusing to overwrite: " + output_dir.string());
  const auto & declarations = array(required(config, "sources", "audit configuration"), "sources"); if (declarations.empty()) throw std::runtime_error("sources must not be empty");
  std::vector<Source> sources; std::set<std::string> ids;
  for (const auto & declaration : declarations) {
    const auto & source_config = object(declaration, "source"); Source source;
    source.id = text(required(source_config, "id", "source"), "source.id"); source.kind = text(required(source_config, "kind", "source"), "source.kind"); source.path = text(required(source_config, "path", "source"), "source.path");
    if (!ids.insert(source.id).second) throw std::runtime_error("duplicate source id: " + source.id);
    const auto absence = absence_status(source_config, &source.reasons, "source");
    std::error_code file_error; const bool regular = fs::is_regular_file(source.path, file_error); const auto size = regular ? fs::file_size(source.path, file_error) : 0U;
    if (file_error || !regular || size == 0U) { if (!absence) { source.result = Status::Fail; source.reasons.push_back(file_error ? "source cannot be read" : "missing or empty required source"); } else { source.result = *absence; source.reasons.push_back("source unavailable: explicit " + std::string(name(*absence))); } sources.push_back(std::move(source)); continue; }
    try {
      source.digest = sha256(source.path); if (const auto expected = optional_text(source_config, "sha256")) { if (!sha256_text(*expected) || source.digest != *expected) source.reasons.push_back("declared SHA-256 mismatch"); }
      const auto root = object(Parser(read_file(source.path)).parse(), "source JSON");
      const auto schema = find(root, "schema_version"); if (!schema || !std::holds_alternative<double>(*schema) || std::get<double>(*schema) != 1.0) source.reasons.push_back("unknown or missing schema_version");
      const auto report_status = find(root, "status"); if (!report_status) source.reasons.push_back("missing report status"); else {
        const auto report_text = text(*report_status, "status");
        if (report_text == "WARN") { source.result = Status::NotVerified; source.reasons.push_back("source reports WARN; cannot count as release PASS"); }
        else { try { const auto reported = status(report_text); if (reported == Status::Fail) source.reasons.push_back("source reports FAIL"); else if (reported != Status::Pass) { source.result = reported; source.reasons.push_back("source reports " + report_text + "; cannot count as release PASS"); } } catch (...) { source.reasons.push_back("unknown report status"); } }
      }
      if (unsafe_claim(Json(root))) source.reasons.push_back("production or hardware claim detected");
      std::string reason; if (!safe_fields(root, &reason)) source.reasons.push_back(reason);
      if (source.kind == "scenario_benchmark" || source.kind == "evidence" || source.kind == "calibration" || source.kind == "qualification") { if (!expected_synthetic(root, &reason)) source.reasons.push_back(reason); }
      if (!counts_valid(root, &reason)) source.reasons.push_back(reason);
      if (const auto ctest_path = optional_text(source_config, "ctest_xml")) {
        if (!ctest_matches_report(root, *ctest_path, &reason)) source.reasons.push_back(reason);
      } else if (find(root, "counts") || find(root, "cases")) {
        source.reasons.push_back("CTest-backed report requires explicit ctest_xml");
      }
      const auto commit_key = source.kind == "ros_e2e" ? "candidate_commit" : "commit";
      if (const auto * commit = find(root, commit_key); commit && text(*commit, commit_key) != head) source.reasons.push_back("candidate Git SHA mismatch");
      if (const auto * source_base = find(root, "baseline_main"); source_base && text(*source_base, "baseline_main") != baseline) source.reasons.push_back("main baseline SHA mismatch");
      if (source.kind == "ros_e2e") { const auto * live = find(root, "node_liveness"); if (!live || !std::holds_alternative<Object>(*live)) { source.result = Status::NotVerified; source.reasons.push_back("node_liveness evidence missing; E2E cannot count as PASS"); } else { const auto & evidence = std::get<Object>(*live); const auto * alive = find(evidence, "alive_during_sampling"); const auto * expected_exit = find(evidence, "expected_exit_code"); const auto * observed_exit = find(evidence, "observed_exit_code"); if (!alive || !expected_exit || !observed_exit || !std::holds_alternative<bool>(*alive) || !std::get<bool>(*alive) || !std::holds_alternative<double>(*expected_exit) || !std::holds_alternative<double>(*observed_exit) || std::get<double>(*expected_exit) != std::get<double>(*observed_exit)) { source.result = Status::NotVerified; source.reasons.push_back("node_liveness evidence contradicts sampling or expected exit"); } } }
      const auto non_fatal_status_reason = [](const std::string & item) {
        return item == "node_liveness evidence missing; E2E cannot count as PASS" ||
               item == "node_liveness evidence contradicts sampling or expected exit" ||
               item == "source reports WARN; cannot count as release PASS" ||
               item.find("source reports UNAVAILABLE; cannot count as release PASS") == 0U ||
               item.find("source reports NOT_RUN; cannot count as release PASS") == 0U ||
               item.find("source reports NOT_VERIFIED; cannot count as release PASS") == 0U;
      };
      if (is_absence_status(source.result)) {
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
      const auto absence = absence_status(item, &artifact.reasons, "artifact"); std::error_code file_error; const bool regular = fs::is_regular_file(artifact.path, file_error); const auto size = regular ? fs::file_size(artifact.path, file_error) : 0U;
      if (file_error || !regular || size == 0U) { artifact.result = absence ? *absence : Status::Fail; artifact.reasons.push_back(absence ? "artifact unavailable: explicit " + std::string(name(*absence)) : "missing or empty required artifact"); } else {
        artifact.digest = sha256(artifact.path); if (const auto expected = optional_text(item, "sha256")) { if (!sha256_text(*expected) || artifact.digest != *expected) artifact.reasons.push_back("declared SHA-256 mismatch"); }
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
