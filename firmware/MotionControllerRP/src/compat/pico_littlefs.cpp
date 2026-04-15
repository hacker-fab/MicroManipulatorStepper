// =============================================================================
// pico_littlefs.cpp — LittleFS File / LittleFSClass implementation backed by
// Pico RP2350 on-board flash.
//
// Flash layout (must match platformio.ini board_build.filesystem_size):
//   Last 0.5 MB of 4 MB flash → offset 0x180000, size 0x80000
//
// Flash operations use hardware_flash (erase/program) and direct XIP reads.
// All flash writes MUST run with interrupts disabled on the calling core and
// must not be called from Core 1 while Core 0 is accessing flash.
// =============================================================================

#ifndef ARDUINO

#include "LittleFS.h"
#include "lfs.h"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include <cstring>
#include <cstdio>

// --- Flash geometry (defaults, overridable via compile defs) -----------------

#ifndef LFS_FLASH_OFFSET
#define LFS_FLASH_OFFSET  0x180000   // 1.5 MB into flash (last 0.5 MB)
#endif
#ifndef LFS_FLASH_SIZE
#define LFS_FLASH_SIZE    0x80000    // 512 KB
#endif

#define LFS_BLOCK_SIZE    FLASH_SECTOR_SIZE   // 4096
#define LFS_BLOCK_COUNT   (LFS_FLASH_SIZE / LFS_BLOCK_SIZE)

// XIP base for direct reads
#ifndef XIP_BASE
#define XIP_BASE 0x10000000
#endif

// --- littlefs block-device callbacks ----------------------------------------

static int lfs_flash_read(const struct lfs_config* c, lfs_block_t block,
                          lfs_off_t off, void* buffer, lfs_size_t size) {
    (void)c;
    uint32_t addr = LFS_FLASH_OFFSET + block * LFS_BLOCK_SIZE + off;
    memcpy(buffer, (const void*)(XIP_BASE + addr), size);
    return LFS_ERR_OK;
}

static int lfs_flash_prog(const struct lfs_config* c, lfs_block_t block,
                          lfs_off_t off, const void* buffer, lfs_size_t size) {
    (void)c;
    uint32_t addr = LFS_FLASH_OFFSET + block * LFS_BLOCK_SIZE + off;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(addr, static_cast<const uint8_t*>(buffer), size);
    restore_interrupts(ints);
    return LFS_ERR_OK;
}

static int lfs_flash_erase(const struct lfs_config* c, lfs_block_t block) {
    (void)c;
    uint32_t addr = LFS_FLASH_OFFSET + block * LFS_BLOCK_SIZE;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(addr, LFS_BLOCK_SIZE);
    restore_interrupts(ints);
    return LFS_ERR_OK;
}

static int lfs_flash_sync(const struct lfs_config* c) {
    (void)c;
    return LFS_ERR_OK;
}

// --- Static buffers for littlefs (avoids heap in config) --------------------

static uint8_t lfs_read_buf[FLASH_PAGE_SIZE];
static uint8_t lfs_prog_buf[FLASH_PAGE_SIZE];
static uint8_t lfs_lookahead_buf[16];

static lfs_config make_lfs_config() {
    lfs_config cfg = {};
    cfg.read        = lfs_flash_read;
    cfg.prog        = lfs_flash_prog;
    cfg.erase       = lfs_flash_erase;
    cfg.sync        = lfs_flash_sync;
    cfg.read_size   = 1;
    cfg.prog_size   = FLASH_PAGE_SIZE;
    cfg.block_size  = LFS_BLOCK_SIZE;
    cfg.block_count = LFS_BLOCK_COUNT;
    cfg.cache_size  = FLASH_PAGE_SIZE;
    cfg.lookahead_size = sizeof(lfs_lookahead_buf);
    cfg.block_cycles   = 500;
    cfg.read_buffer      = lfs_read_buf;
    cfg.prog_buffer      = lfs_prog_buf;
    cfg.lookahead_buffer = lfs_lookahead_buf;
    return cfg;
}

// =============================================================================
// File implementation
// =============================================================================

File::File(lfs_t* lfs, const char* path, int flags)
    : m_lfs(lfs), m_flags(flags) {
    // Determine name (last path component)
    const char* slash = strrchr(path, '/');
    const char* basename = slash ? slash + 1 : path;
    strncpy(m_name, basename, sizeof(m_name) - 1);

    // Try to stat to see if it's a directory
    lfs_info info;
    if (lfs_stat(lfs, path, &info) == LFS_ERR_OK && info.type == LFS_TYPE_DIR) {
        m_is_dir = true;
        int err = lfs_dir_open(lfs, &m_dir, path);
        m_open = (err == LFS_ERR_OK);
    } else {
        m_is_dir = false;
        int err = lfs_file_open(lfs, &m_file, path, flags);
        m_open = (err == LFS_ERR_OK);
    }
}

File::File(File&& o) noexcept
    : m_lfs(o.m_lfs), m_file(o.m_file), m_dir(o.m_dir),
      m_open(o.m_open), m_is_dir(o.m_is_dir), m_flags(o.m_flags) {
    memcpy(m_name, o.m_name, sizeof(m_name));
    o.m_open = false;
}

File& File::operator=(File&& o) noexcept {
    if (this != &o) {
        close_internal();
        m_lfs    = o.m_lfs;
        m_file   = o.m_file;
        m_dir    = o.m_dir;
        m_open   = o.m_open;
        m_is_dir = o.m_is_dir;
        m_flags  = o.m_flags;
        memcpy(m_name, o.m_name, sizeof(m_name));
        o.m_open = false;
    }
    return *this;
}

File::~File() { close_internal(); }

void File::close() { close_internal(); }

void File::close_internal() {
    if (!m_open || !m_lfs) return;
    if (m_is_dir)
        lfs_dir_close(m_lfs, &m_dir);
    else
        lfs_file_close(m_lfs, &m_file);
    m_open = false;
}

size_t File::write(const uint8_t* buf, size_t len) {
    if (!m_open || m_is_dir) return 0;
    lfs_ssize_t written = lfs_file_write(m_lfs, &m_file, buf, len);
    return (written < 0) ? 0 : static_cast<size_t>(written);
}

size_t File::read(uint8_t* buf, size_t len) {
    if (!m_open || m_is_dir) return 0;
    lfs_ssize_t rd = lfs_file_read(m_lfs, &m_file, buf, len);
    return (rd < 0) ? 0 : static_cast<size_t>(rd);
}

File File::openNextFile() {
    if (!m_open || !m_is_dir) return File();

    lfs_info info;
    while (true) {
        int res = lfs_dir_read(m_lfs, &m_dir, &info);
        if (res <= 0) return File(); // end of dir or error

        // Skip . and ..
        if (strcmp(info.name, ".") == 0 || strcmp(info.name, "..") == 0)
            continue;

        // Build full path (we stored the dir path; for simplicity assume root)
        char path[128];
        snprintf(path, sizeof(path), "/%s", info.name);

        int flags = (info.type == LFS_TYPE_DIR) ? LFS_O_RDONLY : LFS_O_RDONLY;
        return File(m_lfs, path, flags);
    }
}

// =============================================================================
// LittleFSClass implementation
// =============================================================================

LittleFSClass LittleFS;

bool LittleFSClass::begin() {
    m_cfg = make_lfs_config();

    int err = lfs_mount(&m_lfs, &m_cfg);
    if (err != LFS_ERR_OK) {
        // First mount after erase — format and retry
        lfs_format(&m_lfs, &m_cfg);
        err = lfs_mount(&m_lfs, &m_cfg);
    }
    m_mounted = (err == LFS_ERR_OK);
    return m_mounted;
}

void LittleFSClass::format() {
    if (m_mounted) {
        lfs_unmount(&m_lfs);
        m_mounted = false;
    }
    m_cfg = make_lfs_config();
    lfs_format(&m_lfs, &m_cfg);
}

File LittleFSClass::open(const char* path, const char* mode) {
    if (!m_mounted) return File();

    int flags = 0;
    if (strcmp(mode, "r") == 0) {
        flags = LFS_O_RDONLY;
    } else if (strcmp(mode, "w") == 0) {
        flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC;
    } else if (strcmp(mode, "a") == 0) {
        flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND;
    } else {
        flags = LFS_O_RDONLY;
    }

    return File(&m_lfs, path, flags);
}

bool LittleFSClass::exists(const char* path) {
    if (!m_mounted) return false;
    lfs_info info;
    return lfs_stat(&m_lfs, path, &info) == LFS_ERR_OK;
}

bool LittleFSClass::info(FSInfo& out) {
    if (!m_mounted) return false;
    lfs_ssize_t used = lfs_fs_size(&m_lfs);
    if (used < 0) return false;
    out.totalBytes = LFS_BLOCK_COUNT * LFS_BLOCK_SIZE;
    out.usedBytes  = static_cast<size_t>(used) * LFS_BLOCK_SIZE;
    return true;
}

#endif // ARDUINO
