#pragma once

namespace owen_common::types {
template <typename T>
class MailboxData {
 public:
  inline MailboxData() : data(), hasNewData(false){};
  inline explicit MailboxData(const T& t) : data(t), hasNewData(true){};
  inline explicit MailboxData(T&& t) : data(t), hasNewData(true){};

  inline T GetData() {
    hasNewData = false;
    return data;
  };

  inline const T& GetDataRef() {
    hasNewData = false;
    return data;
  }

  inline T PeekData() const { return data; }

  inline const T& PeekDataRef() const { return data; }

  inline bool HasNewData() const { return HasNewData(); };

  inline void SetData(const T& t) {
    hasNewData = true;
    data = t;
  }

  inline void SetData(T&& t) {
    hasNewData = true;
    data = t;
  }

  inline void ResetFlag() { hasNewData = false; }

 private:
  T data;
  bool hasNewData;
};
}  // namespace owen_common::types
