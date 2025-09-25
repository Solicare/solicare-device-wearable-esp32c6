#ifndef LOCK_FREE_QUEUE_HPP
#define LOCK_FREE_QUEUE_HPP

#include <atomic>
#include <cstddef>
#include "esp_log.h"

// ���� ������(Single-Producer), ���� �Һ���(Single-Consumer)�� ���� Lock-free ������ ť
// Ư¡:
// - Lock�� ������� �ʾ� high-priority task�� block���� ����
// - ť�� ���� ����, ���� ������ �����͸� ������ ���ο� �����ͷ� ��� (Producer�� ���� ������ ����)
template <typename T, size_t Size>
class LockFreeQueue {
public:
    LockFreeQueue() : m_head(0), m_tail(0) {}

    // ������(Producer)�� ȣ���ϴ� �Լ�
    // �׻� �����ϸ�, ť�� ���� á�� ��� ���� ������ �����͸� ����ϴ�.
    void push(const T& item) {
        const auto current_tail = m_tail.load(std::memory_order_relaxed);
        const auto next_tail = (current_tail + 1) % Size;

        // ť�� ���� á���� Ȯ��
        if (next_tail == m_head.load(std::memory_order_acquire)) {
            // �� á���Ƿ�, �Һ��ڰ� ���� ��ġ(head)�� �� ĭ ������ �̵�����
            // ���� ������ �����͸� �����ϴ�.
            m_head.store((m_head.load(std::memory_order_relaxed) + 1) % Size, std::memory_order_release);
            ESP_LOGW("LockFreeQueue", "Buffer overflow. Oldest data was overwritten.");
        }

        // �� �����͸� ���ۿ� ���ϴ�.
        m_buffer[current_tail] = item;

        // ������ ���Ⱑ �Ϸ�Ǿ����� �Һ��ڿ��� �˸��� ���� tail �����͸� ������Ʈ�մϴ�.
        m_tail.store(next_tail, std::memory_order_release);
    }

    // �Һ���(Consumer)�� ȣ���ϴ� �Լ�
    // ť���� �����͸� �������µ� �����ϸ� true, ť�� ��������� false�� ��ȯ�մϴ�.
    bool pop(T& item) {
        const auto current_head = m_head.load(std::memory_order_relaxed);

        // ť�� ����ִ��� Ȯ��
        if (current_head == m_tail.load(std::memory_order_acquire)) {
            return false; // ť�� �������
        }

        // �����͸� ���ۿ��� �о�ɴϴ�.
        item = m_buffer[current_head];

        // ������ �бⰡ �Ϸ�Ǿ����� �����ڿ��� �˸��� ���� head �����͸� ������Ʈ�մϴ�.
        m_head.store((current_head + 1) % Size, std::memory_order_release);
        return true;
    }

private:
    T m_buffer[Size];
    // std::atomic�� ����Ͽ� ���� �½�ũ���� �����ϰ� ���� ����
    std::atomic<size_t> m_head; // �Һ��ڰ� ������ ���� ��ġ
    std::atomic<size_t> m_tail; // �����ڰ� ������ �� ��ġ
};

#endif // LOCK_FREE_QUEUE_HPP
