#ifndef LOCK_FREE_QUEUE_HPP
#define LOCK_FREE_QUEUE_HPP

#include <atomic>
#include <cstddef>
#include "esp_log.h"

// 단일 생산자(Single-Producer), 단일 소비자(Single-Consumer)를 위한 Lock-free 링버퍼 큐
// 특징:
// - Lock을 사용하지 않아 high-priority task가 block되지 않음
// - 큐가 가득 차면, 가장 오래된 데이터를 버리고 새로운 데이터로 덮어씀 (Producer는 절대 멈추지 않음)
template <typename T, size_t Size>
class LockFreeQueue {
public:
    LockFreeQueue() : m_head(0), m_tail(0) {}

    // 생산자(Producer)가 호출하는 함수
    // 항상 성공하며, 큐가 가득 찼을 경우 가장 오래된 데이터를 덮어씁니다.
    void push(const T& item) {
        const auto current_tail = m_tail.load(std::memory_order_relaxed);
        const auto next_tail = (current_tail + 1) % Size;

        // 큐가 가득 찼는지 확인
        if (next_tail == m_head.load(std::memory_order_acquire)) {
            // 꽉 찼으므로, 소비자가 읽을 위치(head)를 한 칸 강제로 이동시켜
            // 가장 오래된 데이터를 버립니다.
            m_head.store((m_head.load(std::memory_order_relaxed) + 1) % Size, std::memory_order_release);
            ESP_LOGW("LockFreeQueue", "Buffer overflow. Oldest data was overwritten.");
        }

        // 새 데이터를 버퍼에 씁니다.
        m_buffer[current_tail] = item;

        // 데이터 쓰기가 완료되었음을 소비자에게 알리기 위해 tail 포인터를 업데이트합니다.
        m_tail.store(next_tail, std::memory_order_release);
    }

    // 소비자(Consumer)가 호출하는 함수
    // 큐에서 데이터를 가져오는데 성공하면 true, 큐가 비어있으면 false를 반환합니다.
    bool pop(T& item) {
        const auto current_head = m_head.load(std::memory_order_relaxed);

        // 큐가 비어있는지 확인
        if (current_head == m_tail.load(std::memory_order_acquire)) {
            return false; // 큐가 비어있음
        }

        // 데이터를 버퍼에서 읽어옵니다.
        item = m_buffer[current_head];

        // 데이터 읽기가 완료되었음을 생산자에게 알리기 위해 head 포인터를 업데이트합니다.
        m_head.store((current_head + 1) % Size, std::memory_order_release);
        return true;
    }

private:
    T m_buffer[Size];
    // std::atomic을 사용하여 여러 태스크에서 안전하게 접근 가능
    std::atomic<size_t> m_head; // 소비자가 다음에 읽을 위치
    std::atomic<size_t> m_tail; // 생산자가 다음에 쓸 위치
};

#endif // LOCK_FREE_QUEUE_HPP
