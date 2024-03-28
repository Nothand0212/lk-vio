#pragma once

namespace lk_vio
{
  template <typename T>
  class Singleton
  {
  public:
    static T& getInstance()
    {
      static T instance;  // Guaranteed to be destroyed, instantiated on first use.
      return instance;
    }

  protected:
    Singleton()                   = default;
    virtual ~Singleton()          = default;
    Singleton( const Singleton& ) = delete;
    Singleton& operator=( const Singleton& ) = delete;
  };
}  // namespace lk_vio