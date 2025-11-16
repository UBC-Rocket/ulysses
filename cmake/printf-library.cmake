include(FetchContent)

FetchContent_Declare(
    printf
    GIT_REPOSITORY https://github.com/eyalroz/printf.git
    GIT_TAG f1b728cbd5c6e10dc1f140f1574edfd1ccdcbedb # v6.3.0
    GIT_SHALLOW ON
)
