#ifndef ULYSSES_COMPILER
#define ULYSSES_COMPILER

#ifdef __GNUC__

#ifndef ULYSSES_WEAK
#define ULYSSES_WEAK __attribute__((weak))
#endif // ULYSSES_WEAK

#ifndef ULYSSES_ALIGNED
#define ULYSSES_ALIGNED(alignment) __attribute__((aligned(alignment)))
#endif // ULYSSES_ALIGNED

#ifndef ULYSSES_NORETURN
#define ULYSSES_NORETURN __attribute__((noreturn))
#endif // ULYSSES_NORETURN

#ifndef ULYSSES_UNUSED
#define ULYSSES_UNUSED __attribute__((unused))
#endif // ULYSSES_UNUSED

#endif // __GNUC__

#endif // ULYSSES_COMPILER
