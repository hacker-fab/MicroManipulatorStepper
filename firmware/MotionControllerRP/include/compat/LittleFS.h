// =============================================================================
// LittleFS.h — Arduino-compatible LittleFS wrapper over raw littlefs + Pico
//              flash storage.  Implements the subset used by MotionControllerRP:
//                File (read/write/close/name/isDirectory/openNextFile)
//                LittleFSClass (begin/format/open/info/exists)
//                FSInfo
// =============================================================================
#pragma once

#include "lfs.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

// ---- FSInfo (mirrors Arduino ESP/RP2040 FSInfo) ----------------------------

struct FSInfo {
    size_t totalBytes = 0;
    size_t usedBytes  = 0;
};

// ---- File ------------------------------------------------------------------

class File {
public:
    File() = default;

    // Internal constructor used by LittleFSClass::open / openNextFile
    File(lfs_t* lfs, const char* path, int flags);

    // Movable, not copyable
    File(File&& o) noexcept;
    File& operator=(File&& o) noexcept;
    File(const File&)            = delete;
    File& operator=(const File&) = delete;

    ~File();

    explicit operator bool() const { return m_open; }

    size_t write(const uint8_t* buf, size_t len);
    size_t read(uint8_t* buf, size_t len);
    void   close();

    bool        isDirectory() const { return m_is_dir; }
    const char* name()        const { return m_name; }

    // Iterate directory entries (Arduino-style)
    File openNextFile();

private:
    void close_internal();

    lfs_t*    m_lfs    = nullptr;
    lfs_file_t m_file  = {};
    lfs_dir_t  m_dir   = {};
    bool      m_open   = false;
    bool      m_is_dir = false;
    int       m_flags  = 0;
    char      m_name[64] = {};
};

// ---- LittleFSClass ---------------------------------------------------------

class LittleFSClass {
public:
    bool begin();
    void format();

    File open(const char* path, const char* mode);
    bool exists(const char* path);
    bool info(FSInfo& out);

    // Access the raw lfs handle (for advanced use)
    lfs_t* lfs() { return &m_lfs; }

private:
    lfs_t        m_lfs     = {};
    lfs_config   m_cfg     = {};
    bool         m_mounted = false;
};

extern LittleFSClass LittleFS;
