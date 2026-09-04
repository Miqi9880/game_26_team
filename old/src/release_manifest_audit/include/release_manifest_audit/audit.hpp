#pragma once

#include <filesystem>
#include <iosfwd>

namespace release_manifest_audit
{

// The configuration is JSON schema v1.  Every source and artifact path is
// explicit; this tool never guesses an artifact from a filename.
int run(const std::filesystem::path & config_path, const std::filesystem::path & output_dir,
        std::ostream & output, std::ostream & error);

}  // namespace release_manifest_audit
