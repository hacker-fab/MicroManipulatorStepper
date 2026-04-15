// Tests for RingBuffer<T, N> (ringbuffer.h)
#include <gtest/gtest.h>
#include "utilities/ringbuffer.h"

// ---------------------------------------------------------------------------
// Basic push/pop
// ---------------------------------------------------------------------------
TEST(RingBuffer, InitiallyEmpty) {
    RingBuffer<int, 4> rb;
    EXPECT_TRUE(rb.empty());
    EXPECT_FALSE(rb.full());
    EXPECT_EQ(rb.size(), 0);
}

TEST(RingBuffer, PushIncreasesSize) {
    RingBuffer<int, 4> rb;
    rb.push(1);
    EXPECT_EQ(rb.size(), 1);
    EXPECT_FALSE(rb.empty());
}

TEST(RingBuffer, PopDecreasesSize) {
    RingBuffer<int, 4> rb;
    rb.push(42);
    int val = 0;
    EXPECT_TRUE(rb.pop(val));
    EXPECT_EQ(val, 42);
    EXPECT_TRUE(rb.empty());
}

TEST(RingBuffer, FIFOOrder) {
    RingBuffer<int, 8> rb;
    for (int i = 0; i < 5; ++i) rb.push(i * 10);
    for (int i = 0; i < 5; ++i) {
        int val = -1;
        EXPECT_TRUE(rb.pop(val));
        EXPECT_EQ(val, i * 10);
    }
}

// ---------------------------------------------------------------------------
// Full / overflow behaviour
// ---------------------------------------------------------------------------
TEST(RingBuffer, FullWhenCapacityReached) {
    RingBuffer<int, 3> rb;
    rb.push(1); rb.push(2); rb.push(3);
    EXPECT_TRUE(rb.full());
    EXPECT_EQ(rb.size(), 3);
    EXPECT_EQ(rb.free_item_count(), 0);
}

TEST(RingBuffer, PushOnFullReturnsNullptr) {
    RingBuffer<int, 2> rb;
    rb.push(1); rb.push(2);
    EXPECT_EQ(rb.push(3), nullptr);
}

TEST(RingBuffer, PushOnFullDoesNotChangeSize) {
    RingBuffer<int, 2> rb;
    rb.push(1); rb.push(2);
    rb.push(3);  // should be rejected
    EXPECT_EQ(rb.size(), 2);
}

// ---------------------------------------------------------------------------
// Empty / underflow behaviour
// ---------------------------------------------------------------------------
TEST(RingBuffer, PopOnEmptyReturnsFalse) {
    RingBuffer<int, 4> rb;
    int val = 0;
    EXPECT_FALSE(rb.pop(val));
    EXPECT_EQ(val, 0);  // unchanged
}

TEST(RingBuffer, DiscardPopOnEmptyReturnsFalse) {
    RingBuffer<int, 4> rb;
    EXPECT_FALSE(rb.pop());
}

// ---------------------------------------------------------------------------
// peek
// ---------------------------------------------------------------------------
TEST(RingBuffer, PeekReturnsHeadWithoutRemoving) {
    RingBuffer<int, 4> rb;
    rb.push(10); rb.push(20);
    EXPECT_EQ(*rb.peek(), 10);
    EXPECT_EQ(rb.size(), 2);  // still 2
}

TEST(RingBuffer, PeekOnEmptyReturnsNullptr) {
    RingBuffer<int, 4> rb;
    EXPECT_EQ(rb.peek(), nullptr);
}

// ---------------------------------------------------------------------------
// get(i)
// ---------------------------------------------------------------------------
TEST(RingBuffer, GetByIndexReturnsCorrectElement) {
    RingBuffer<int, 8> rb;
    rb.push(100); rb.push(200); rb.push(300);
    EXPECT_EQ(*rb.get(0), 100);
    EXPECT_EQ(*rb.get(1), 200);
    EXPECT_EQ(*rb.get(2), 300);
}

// ---------------------------------------------------------------------------
// Wrap-around: fill, drain, then fill again
// ---------------------------------------------------------------------------
TEST(RingBuffer, WrapAroundMaintainsFIFO) {
    RingBuffer<int, 4> rb;
    // Fill to capacity
    rb.push(1); rb.push(2); rb.push(3); rb.push(4);
    // Drain two
    int a, b;
    rb.pop(a); rb.pop(b);
    EXPECT_EQ(a, 1); EXPECT_EQ(b, 2);
    // Push two more — internal head wraps
    rb.push(5); rb.push(6);
    // Drain all four remaining
    int c, d, e, f;
    rb.pop(c); rb.pop(d); rb.pop(e); rb.pop(f);
    EXPECT_EQ(c, 3); EXPECT_EQ(d, 4); EXPECT_EQ(e, 5); EXPECT_EQ(f, 6);
}

// ---------------------------------------------------------------------------
// free_item_count
// ---------------------------------------------------------------------------
TEST(RingBuffer, FreeItemCount) {
    RingBuffer<int, 5> rb;
    EXPECT_EQ(rb.free_item_count(), 5);
    rb.push(1); rb.push(2);
    EXPECT_EQ(rb.free_item_count(), 3);
}

// ---------------------------------------------------------------------------
// Works with non-trivial types
// ---------------------------------------------------------------------------
struct Point { float x, y; };

TEST(RingBuffer, WorksWithStructType) {
    RingBuffer<Point, 4> rb;
    rb.push({1.0f, 2.0f});
    rb.push({3.0f, 4.0f});
    Point p{};
    rb.pop(p);
    EXPECT_FLOAT_EQ(p.x, 1.0f);
    EXPECT_FLOAT_EQ(p.y, 2.0f);
}
