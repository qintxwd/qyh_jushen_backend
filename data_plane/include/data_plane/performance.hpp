/**
 * @file performance.hpp
 * @brief 性能优化模块
 * 
 * 包含：
 * - 消息序列化缓存
 * - 内存池分配
 * - 零拷贝支持
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <queue>
#include <functional>
#include <cstring>

namespace qyh::dataplane {

// ==================== 内存池 ====================

/**
 * @brief 固定大小内存块池
 */
class MemoryPool {
public:
    explicit MemoryPool(size_t block_size, size_t initial_blocks = 64)
        : block_size_(block_size)
    {
        expand(initial_blocks);
    }
    
    ~MemoryPool() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (void* block : all_blocks_) {
            ::operator delete(block);
        }
    }
    
    /**
     * @brief 分配一个内存块
     */
    void* allocate() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (free_blocks_.empty()) {
            expand(all_blocks_.size()); // 翻倍扩展
        }
        
        void* block = free_blocks_.front();
        free_blocks_.pop();
        ++allocated_count_;
        return block;
    }
    
    /**
     * @brief 释放内存块回池
     */
    void deallocate(void* ptr) {
        if (!ptr) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        free_blocks_.push(ptr);
        --allocated_count_;
    }
    
    /**
     * @brief 获取统计信息
     */
    struct Stats {
        size_t block_size;
        size_t total_blocks;
        size_t free_blocks;
        size_t allocated_blocks;
    };
    
    Stats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {
            block_size_,
            all_blocks_.size(),
            free_blocks_.size(),
            allocated_count_
        };
    }
    
private:
    void expand(size_t count) {
        for (size_t i = 0; i < count; ++i) {
            void* block = ::operator new(block_size_);
            all_blocks_.push_back(block);
            free_blocks_.push(block);
        }
    }
    
    size_t block_size_;
    std::vector<void*> all_blocks_;
    std::queue<void*> free_blocks_;
    size_t allocated_count_ = 0;
    mutable std::mutex mutex_;
};

/**
 * @brief 多大小内存池管理器
 */
class PooledAllocator {
public:
    static PooledAllocator& instance() {
        static PooledAllocator instance;
        return instance;
    }
    
    /**
     * @brief 分配内存
     */
    void* allocate(size_t size) {
        size_t pool_size = find_pool_size(size);
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = pools_.find(pool_size);
        if (it == pools_.end()) {
            it = pools_.emplace(pool_size, std::make_unique<MemoryPool>(pool_size)).first;
        }
        return it->second->allocate();
    }
    
    /**
     * @brief 释放内存
     */
    void deallocate(void* ptr, size_t size) {
        size_t pool_size = find_pool_size(size);
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = pools_.find(pool_size);
        if (it != pools_.end()) {
            it->second->deallocate(ptr);
        }
    }
    
private:
    PooledAllocator() = default;
    
    // 向上取整到池大小（64, 128, 256, 512, 1024, 2048, 4096, ...）
    size_t find_pool_size(size_t size) {
        if (size <= 64) return 64;
        size_t pool_size = 64;
        while (pool_size < size) {
            pool_size *= 2;
        }
        return pool_size;
    }
    
    std::unordered_map<size_t, std::unique_ptr<MemoryPool>> pools_;
    std::mutex mutex_;
};

/**
 * @brief 池化缓冲区
 */
class PooledBuffer {
public:
    explicit PooledBuffer(size_t size = 0) : size_(size) {
        if (size > 0) {
            data_ = static_cast<char*>(PooledAllocator::instance().allocate(size));
            capacity_ = size;
        }
    }
    
    ~PooledBuffer() {
        if (data_) {
            PooledAllocator::instance().deallocate(data_, capacity_);
        }
    }
    
    // 移动语义
    PooledBuffer(PooledBuffer&& other) noexcept
        : data_(other.data_), size_(other.size_), capacity_(other.capacity_)
    {
        other.data_ = nullptr;
        other.size_ = 0;
        other.capacity_ = 0;
    }
    
    PooledBuffer& operator=(PooledBuffer&& other) noexcept {
        if (this != &other) {
            if (data_) {
                PooledAllocator::instance().deallocate(data_, capacity_);
            }
            data_ = other.data_;
            size_ = other.size_;
            capacity_ = other.capacity_;
            other.data_ = nullptr;
            other.size_ = 0;
            other.capacity_ = 0;
        }
        return *this;
    }
    
    // 禁止拷贝
    PooledBuffer(const PooledBuffer&) = delete;
    PooledBuffer& operator=(const PooledBuffer&) = delete;
    
    char* data() { return data_; }
    const char* data() const { return data_; }
    size_t size() const { return size_; }
    size_t capacity() const { return capacity_; }
    
    void resize(size_t new_size) {
        if (new_size > capacity_) {
            char* new_data = static_cast<char*>(
                PooledAllocator::instance().allocate(new_size));
            if (data_) {
                std::memcpy(new_data, data_, size_);
                PooledAllocator::instance().deallocate(data_, capacity_);
            }
            data_ = new_data;
            capacity_ = new_size;
        }
        size_ = new_size;
    }
    
    void write(const void* src, size_t len) {
        if (size_ + len > capacity_) {
            resize(std::max(capacity_ * 2, size_ + len));
        }
        std::memcpy(data_ + size_, src, len);
        size_ += len;
    }
    
private:
    char* data_ = nullptr;
    size_t size_ = 0;
    size_t capacity_ = 0;
};

// ==================== 序列化缓存 ====================

/**
 * @brief 序列化缓存条目
 */
struct CacheEntry {
    std::string serialized_data;
    size_t hash = 0;
    std::chrono::steady_clock::time_point last_update;
    uint64_t hit_count = 0;
};

/**
 * @brief 消息序列化缓存
 */
class SerializationCache {
public:
    explicit SerializationCache(size_t max_entries = 1000, 
                                 std::chrono::seconds ttl = std::chrono::seconds(5))
        : max_entries_(max_entries), ttl_(ttl)
    {}
    
    /**
     * @brief 获取或创建缓存条目
     * 
     * @param key 缓存键
     * @param data_hash 原始数据哈希值
     * @param serializer 序列化函数（仅在缓存未命中时调用）
     * @return 序列化后的数据
     */
    const std::string& get_or_create(
        const std::string& key,
        size_t data_hash,
        std::function<std::string()> serializer)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::steady_clock::now();
        auto it = cache_.find(key);
        
        // 缓存命中且数据未变化且未过期
        if (it != cache_.end() && 
            it->second.hash == data_hash &&
            now - it->second.last_update < ttl_) {
            ++it->second.hit_count;
            ++stats_.hits;
            return it->second.serialized_data;
        }
        
        // 缓存未命中或需要更新
        ++stats_.misses;
        
        CacheEntry entry;
        entry.serialized_data = serializer();
        entry.hash = data_hash;
        entry.last_update = now;
        entry.hit_count = 1;
        
        // LRU 淘汰
        if (cache_.size() >= max_entries_) {
            evict_oldest();
        }
        
        cache_[key] = std::move(entry);
        return cache_[key].serialized_data;
    }
    
    /**
     * @brief 使缓存失效
     */
    void invalidate(const std::string& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        cache_.erase(key);
    }
    
    /**
     * @brief 清空缓存
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        cache_.clear();
    }
    
    /**
     * @brief 获取统计信息
     */
    struct Stats {
        size_t entries;
        uint64_t hits;
        uint64_t misses;
        double hit_rate() const { 
            auto total = hits + misses;
            return total > 0 ? static_cast<double>(hits) / total : 0; 
        }
    };
    
    Stats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {cache_.size(), stats_.hits, stats_.misses};
    }
    
private:
    void evict_oldest() {
        if (cache_.empty()) return;
        
        auto oldest = cache_.begin();
        for (auto it = cache_.begin(); it != cache_.end(); ++it) {
            if (it->second.last_update < oldest->second.last_update) {
                oldest = it;
            }
        }
        cache_.erase(oldest);
        ++stats_.evictions;
    }
    
    size_t max_entries_;
    std::chrono::seconds ttl_;
    std::unordered_map<std::string, CacheEntry> cache_;
    
    struct {
        uint64_t hits = 0;
        uint64_t misses = 0;
        uint64_t evictions = 0;
    } stats_;
    
    mutable std::mutex mutex_;
};

// ==================== 零拷贝支持 ====================

/**
 * @brief 零拷贝缓冲区视图
 */
class BufferView {
public:
    BufferView() = default;
    BufferView(const char* data, size_t size) : data_(data), size_(size) {}
    BufferView(const std::string& str) : data_(str.data()), size_(str.size()) {}
    BufferView(const std::vector<char>& vec) : data_(vec.data()), size_(vec.size()) {}
    
    const char* data() const { return data_; }
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    
    BufferView subview(size_t offset, size_t len = std::string::npos) const {
        if (offset >= size_) return BufferView();
        len = std::min(len, size_ - offset);
        return BufferView(data_ + offset, len);
    }
    
    std::string to_string() const { return std::string(data_, size_); }
    
private:
    const char* data_ = nullptr;
    size_t size_ = 0;
};

/**
 * @brief 共享缓冲区（引用计数）
 */
class SharedBuffer {
public:
    static std::shared_ptr<SharedBuffer> create(size_t size) {
        return std::shared_ptr<SharedBuffer>(new SharedBuffer(size));
    }
    
    static std::shared_ptr<SharedBuffer> create(const std::string& data) {
        auto buf = create(data.size());
        std::memcpy(buf->data(), data.data(), data.size());
        return buf;
    }
    
    char* data() { return data_.data(); }
    const char* data() const { return data_.data(); }
    size_t size() const { return data_.size(); }
    
    BufferView view() const { return BufferView(data_.data(), data_.size()); }
    
private:
    explicit SharedBuffer(size_t size) : data_(size) {}
    
    std::vector<char> data_;
};

// ==================== 批量消息处理 ====================

/**
 * @brief 批量消息聚合器
 */
template<typename T>
class MessageBatcher {
public:
    explicit MessageBatcher(size_t batch_size = 100,
                            std::chrono::milliseconds flush_interval = std::chrono::milliseconds(50))
        : batch_size_(batch_size)
        , flush_interval_(flush_interval)
        , last_flush_(std::chrono::steady_clock::now())
    {}
    
    /**
     * @brief 添加消息
     * 
     * @return true 如果需要刷新批次
     */
    bool add(T message) {
        std::lock_guard<std::mutex> lock(mutex_);
        batch_.push_back(std::move(message));
        
        auto now = std::chrono::steady_clock::now();
        return batch_.size() >= batch_size_ || 
               now - last_flush_ >= flush_interval_;
    }
    
    /**
     * @brief 获取并清空批次
     */
    std::vector<T> flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<T> result = std::move(batch_);
        batch_.clear();
        last_flush_ = std::chrono::steady_clock::now();
        return result;
    }
    
    /**
     * @brief 检查是否需要刷新
     */
    bool should_flush() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();
        return batch_.size() >= batch_size_ || 
               (!batch_.empty() && now - last_flush_ >= flush_interval_);
    }
    
    size_t pending_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return batch_.size();
    }
    
private:
    size_t batch_size_;
    std::chrono::milliseconds flush_interval_;
    std::vector<T> batch_;
    std::chrono::steady_clock::time_point last_flush_;
    mutable std::mutex mutex_;
};

} // namespace qyh::dataplane
