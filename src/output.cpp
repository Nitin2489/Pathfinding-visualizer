#include "output.h"

#include <cctype>
#include <filesystem>
#include <system_error>

namespace fs = std::filesystem;

namespace {

// Comparison key for a path: absolute, lexically normal, generic separators,
// ASCII-case-folded where file systems are usually case-insensitive.
std::string pathKey(const std::string& path) {
    std::error_code ec;
    fs::path p = fs::absolute(fs::path(path), ec);
    if (ec) p = fs::path(path);
    std::string key = p.lexically_normal().generic_string();
#if defined(_WIN32) || defined(__APPLE__)
    for (char& c : key) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
#endif
    return key;
}

}  // namespace

StagedFile::~StagedFile() {
    if (!retain_) discard();
}

std::string StagedFile::partialPath() const { return partialPathFor(path_); }

bool StagedFile::open(const std::string& path, std::string& error) {
    path_ = path;
    stream_.open(partialPath(), std::ios::out | std::ios::trunc);
    if (!stream_) {
        error = "cannot open '" + partialPath() + "' for writing";
        return false;
    }
    created_ = true;
    return true;
}

void StagedFile::discard() {
    if (!created_ || committed_) return;
    if (stream_.is_open()) stream_.close();
    std::error_code ec;
    fs::remove(partialPath(), ec);  // best effort; only the staging file
    created_ = false;
}

bool StagedFile::commit(std::string& error) {
    if (!stream_.is_open()) {
        error = "'" + path_ + "' was not opened";
        return false;
    }
    stream_.flush();
    const bool writeFailed = stream_.fail();
    stream_.close();
    if (writeFailed || stream_.fail()) {
        discard();  // incomplete; the destination is left untouched
        error = "failed while writing '" + partialPath() + "' (incomplete staging file removed)";
        return false;
    }
    std::error_code ec;
    fs::rename(partialPath(), path_, ec);  // replaces an existing file
    if (ec) {
        retain_ = true;  // the staging file is complete: keep it for recovery
        error = "cannot rename '" + partialPath() + "' to '" + path_ + "': " + ec.message() +
                " (complete data kept in the staging file)";
        return false;
    }
    committed_ = true;
    return true;
}

std::string metadataPathFor(const std::string& path) { return path + ".meta.json"; }

std::string partialPathFor(const std::string& path) { return path + ".partial"; }

std::vector<PlannedFile> plannedOutputFiles(const std::vector<OutputRequest>& requests) {
    std::vector<PlannedFile> files;
    for (const OutputRequest& r : requests) {
        const std::string meta = metadataPathFor(r.path);
        files.push_back({r.path, r.option + " destination"});
        files.push_back({partialPathFor(r.path), r.option + " staging file"});
        files.push_back({meta, r.option + " metadata"});
        files.push_back({partialPathFor(meta), r.option + " metadata staging file"});
    }
    return files;
}

bool checkOutputPathCollisions(const std::vector<OutputRequest>& requests, std::string& error) {
    const std::vector<PlannedFile> files = plannedOutputFiles(requests);
    std::vector<std::string> keys;
    keys.reserve(files.size());
    for (const PlannedFile& f : files) keys.push_back(pathKey(f.path));
    for (std::size_t i = 0; i < files.size(); ++i) {
        for (std::size_t j = i + 1; j < files.size(); ++j) {
            if (keys[i] == keys[j]) {
                error = "output path collision: " + files[i].description + " '" + files[i].path + "' and " +
                        files[j].description + " '" + files[j].path + "' are the same file";
                return false;
            }
        }
    }
    return true;
}

bool checkOutputsWritable(const std::vector<std::string>& paths, bool force, std::string& error) {
    if (force) return true;
    for (const std::string& p : paths) {
        const std::string meta = metadataPathFor(p);
        for (const std::string& candidate : {p, meta}) {
            std::error_code ec;
            if (fs::exists(candidate, ec)) {
                error = "'" + candidate + "' already exists (use --force to overwrite)";
                return false;
            }
        }
        // A leftover staging file may hold rows from an interrupted run; do
        // not truncate it silently.
        for (const std::string& candidate : {partialPathFor(p), partialPathFor(meta)}) {
            std::error_code ec;
            if (fs::exists(candidate, ec)) {
                error = "'" + candidate + "' already exists (possibly left by an interrupted run); move it away "
                        "or use --force";
                return false;
            }
        }
    }
    return true;
}
