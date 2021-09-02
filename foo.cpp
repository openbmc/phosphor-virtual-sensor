#include <phosphor-logging/lg2.hpp>
#include <phosphor-logging/log.hpp>
#include <string>

using namespace std::string_literals;
auto s = "hello"s;

void foo()
{
    PHOSPHOR_LOG2_USING;

    error("Hello", "NAME", s, "HELLO", 123);
}

void bar()
{
    using namespace phosphor::logging;

    log<level::ERR>("Hello", entry("NAME=%s", s.c_str()), entry("HELLO=%d", 123));
}
