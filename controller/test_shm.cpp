#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h> // For mode constants
#include <fcntl.h>    // For O_* constants
#include <unistd.h>
#include <cstring>    // For memset

int main() {
    // Name for the shared memory object
    const char *shm_name = "/shared_memory_example";

    // Create or open a shared memory object
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return 1;
    }

    // Set the size of the shared memory object
    if (ftruncate(shm_fd, sizeof(double)) == -1) {
        perror("ftruncate");
        return 1;
    }

    // Map the shared memory object into the process's address space
    double *shared_force = (double *)mmap(NULL, sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_force == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    // Optionally initialize the shared memory (only needed if you're the writer)
    *shared_force = 0.0;

    // Loop to read the shared memory value
    while (true) {
        std::cout << "Control Force: " << *shared_force << std::endl;
        usleep(100000); // Sleep for 100ms
    }

    // Cleanup (not reached in this example due to infinite loop)
    munmap(shared_force, sizeof(double));
    close(shm_fd);
    shm_unlink(shm_name);

    return 0;
}
