#ifndef ULYSSES_STATUS
#define ULYSSES_STATUS

enum uls_status {
    ULS_STATUS_OK,       //< Success
    ULS_STATUS_ERROR,    //< Unknown error
    ULS_STATUS_BUSY,     //< Resource is busy
    ULS_STATUS_TIMEOUT,  //< Operation timed out
    ULS_STATUS_NOT_READY //< Resource is not ready
};

#endif // ULYSSES_STATUS
