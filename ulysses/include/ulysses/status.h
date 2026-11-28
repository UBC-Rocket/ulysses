#ifndef ULYSSES_STATUS
#define ULYSSES_STATUS

enum uls_status {
    ULS_STATUS_OK,           ///< Success
    ULS_STATUS_ERROR,        ///< Unknown error
    ULS_STATUS_BUSY,         ///< Resource is busy
    ULS_STATUS_TIMEOUT,      ///< Operation timed out
    ULS_STATUS_NOT_READY,    ///< Resource is not ready
    ULS_STATUS_NO_SPACE,     ///< Output buffer is too small
    ULS_STATUS_INVALID_INPUT ///< Input data is malformed
};

#endif // ULYSSES_STATUS
