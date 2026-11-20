// Crash-safe output files for the benchmark.
//
// A StagedFile writes to "<path>.partial" and only replaces <path> when
// commit() succeeds, so an existing <path> is never truncated by a run that
// later fails, and a final <path> always holds a complete file.
//
// Cleanup: a StagedFile that is destroyed without commit() removes its own
// .partial file (ordinary failure paths such as a later output failing to
// open), unless retain() was called. Only files this object created are
// removed; final destinations are never deleted. A crash or kill still
// leaves .partial files behind.
//
// Outputs are committed one at a time. There is no multi-file transaction:
// if a later commit fails, earlier outputs have already been replaced.
#pragma once

#include <fstream>
#include <string>
#include <vector>

class StagedFile {
public:
    StagedFile() = default;
    ~StagedFile();
    StagedFile(const StagedFile&) = delete;
    StagedFile& operator=(const StagedFile&) = delete;

    // Opens "<path>.partial" for writing (truncating any stale one).
    bool open(const std::string& path, std::string& error);
    bool isOpen() const { return stream_.is_open(); }
    std::ofstream& stream() { return stream_; }
    const std::string& path() const { return path_; }
    std::string partialPath() const;

    // Flushes, closes, checks for write errors and renames the partial file
    // over <path>. On a write error the (incomplete) partial file is removed;
    // if only the rename fails, the complete partial file is kept. Either
    // way error is set.
    bool commit(std::string& error);

    // Keep the partial file when this object is destroyed uncommitted (used
    // to preserve completed per-run rows after an aborted benchmark).
    void retain() { retain_ = true; }
    // Closes and removes the partial file if this object created it and has
    // not committed it. Never touches the final destination.
    void discard();

private:
    std::string path_;
    std::ofstream stream_;
    bool created_ = false;    // open() succeeded, so the partial file is ours
    bool committed_ = false;
    bool retain_ = false;
};

// Sidecar metadata path for an output file.
std::string metadataPathFor(const std::string& path);
// Staging path used while writing `path`.
std::string partialPathFor(const std::string& path);

// One requested output, e.g. {"--csv", "runs.csv"}.
struct OutputRequest {
    std::string option;
    std::string path;
};

// A file the benchmark will create for a requested output.
struct PlannedFile {
    std::string path;
    std::string description;  // e.g. "--csv metadata"
};

// Every file a run writes for `requests`: per output its destination, its
// staging file, its metadata sidecar and the sidecar's staging file.
std::vector<PlannedFile> plannedOutputFiles(const std::vector<OutputRequest>& requests);

// Fails if any two planned files refer to the same location. Paths are
// compared after making them absolute and lexically normal, so "a.csv",
// "./a.csv" and "dir/../a.csv" collide; on Windows and macOS (usually
// case-insensitive file systems) ASCII case is ignored as well. Symbolic
// links and hard links are not resolved. Touches no files.
bool checkOutputPathCollisions(const std::vector<OutputRequest>& requests, std::string& error);

// Fails (with a message) if, without `force`, any output, its metadata
// sidecar, or either staging file already exists.
bool checkOutputsWritable(const std::vector<std::string>& paths, bool force, std::string& error);
