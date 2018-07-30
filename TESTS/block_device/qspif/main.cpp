#include "mbed.h"

using namespace mbed;

#include "greentea-client/test_env.h"
#include "unity.h"
#include "utest.h"
#include "QSPIFBlockDevice.h"
#include <stdlib.h>

using namespace utest::v1;

#define TEST_BLOCK_COUNT 10
#define TEST_ERROR_MASK 16

const struct {
    const char *name;
    bd_size_t (BlockDevice::*method)() const;
} ATTRS[] = {
    {"read size",    &BlockDevice::get_read_size},
    {"program size", &BlockDevice::get_program_size},
    {"erase size",   &BlockDevice::get_erase_size},
    {"total size",   &BlockDevice::size},
};

static SingletonPtr<PlatformMutex> _mutex;


// Mutex is protecting rand() per srand for buffer writing and verification.
// Mutex is also protecting printouts for clear logs.
// Mutex is NOT protecting Block Device actions: erase/program/read - which is the purpose of the multithreaded test!
void basic_erase_program_read_test(
    QSPIFBlockDevice& blockD,
    bd_size_t block_size,
    uint8_t *write_block,
    uint8_t *read_block,
    uint8_t *error_mask,
    unsigned addrwidth)
{
    int err = 0;
    // Find a random block
    bd_addr_t block = (rand() * block_size) % blockD.size();

    // Use next random number as temporary seed to keep
    // the address progressing in the pseudorandom sequence
    unsigned seed = rand();

    // Fill with random sequence
    _mutex->lock();
    srand(seed);
    for (bd_size_t i_ind = 0; i_ind < block_size; i_ind++) {
        write_block[i_ind] = 0xff & rand();
    }
    // Write, sync, and read the block
    printf("\ntest  %0*llx:%llu...", addrwidth, block, block_size);
    _mutex->unlock();

    err = blockD.erase(block, block_size);
    TEST_ASSERT_EQUAL(0, err);

    err = blockD.program(write_block, block, block_size);
    TEST_ASSERT_EQUAL(0, err);

    _mutex->lock();
    printf("\nwrite %0*llx:%llu ", addrwidth, block, block_size);
    for (int i_ind = 0; i_ind < 16; i_ind++) {
        printf("%02x", write_block[i_ind]);
    }
    printf("...\n");
    _mutex->unlock();

    err = blockD.read(read_block, block, block_size);
    TEST_ASSERT_EQUAL(0, err);

    _mutex->lock();
    printf("read  %0*llx:%llu ", addrwidth, block, block_size);
    for (int i_ind = 0; i_ind < 16; i_ind++) {
        printf("%02x", read_block[i_ind]);
    }
    printf("...\n");
    _mutex->unlock();

    // Find error mask for debugging
    memset(error_mask, 0, TEST_ERROR_MASK);
    bd_size_t error_scale = block_size / (TEST_ERROR_MASK * 8);

    _mutex->lock();
    srand(seed);
    for (bd_size_t i_ind = 0; i_ind < TEST_ERROR_MASK * 8; i_ind++) {
        for (bd_size_t j_ind = 0; j_ind < error_scale; j_ind++) {
            if ((0xff & rand()) != read_block[i_ind * error_scale + j_ind]) {
                error_mask[i_ind / 8] |= 1 << (i_ind % 8);
            }
        }
    }

    printf("error %0*llx:%llu ", addrwidth, block, block_size);
    for (int i_ind = 0; i_ind < 16; i_ind++) {
        printf("%02x", error_mask[i_ind]);
    }
    printf("\n");

    // Check that the data was unmodified
    srand(seed);
    for (bd_size_t i_ind = 0; i_ind < block_size; i_ind++) {
        TEST_ASSERT_EQUAL(0xff & rand(), read_block[i_ind]);
    }
    _mutex->unlock();
}

void test_qspif_random_program_read_erase()
{

    printf("\nTest Random Program Read Erase Starts..\n");

    QSPIFBlockDevice blockD(MBED_CONF_QSPIF_QSPI_IO0, MBED_CONF_QSPIF_QSPI_IO1, MBED_CONF_QSPIF_QSPI_IO2,
                            MBED_CONF_QSPIF_QSPI_IO3, MBED_CONF_QSPIF_QSPI_CLK, MBED_CONF_QSPIF_QSPI_CS, 0, MBED_CONF_QSPIF_QSPI_FREQ);

    int err = blockD.init();
    TEST_ASSERT_EQUAL(0, err);

    for (unsigned atr = 0; atr < sizeof(ATTRS) / sizeof(ATTRS[0]); atr++) {
        static const char *prefixes[] = {"", "k", "M", "G"};
        for (int i_ind = 3; i_ind >= 0; i_ind--) {
            bd_size_t size = (blockD.*ATTRS[atr].method)();
            if (size >= (1ULL << 10 * i_ind)) {
                printf("%s: %llu%sbytes (%llubytes)\n",
                       ATTRS[atr].name, size >> 10 * i_ind, prefixes[i_ind], size);
                break;
            }
        }
    }

    bd_size_t block_size = blockD.get_erase_size();
    uint8_t *write_block = new uint8_t[block_size];
    uint8_t *read_block = new uint8_t[block_size];
    uint8_t *error_mask = new uint8_t[TEST_ERROR_MASK];
    unsigned addrwidth = ceil(log(float(blockD.size() - 1)) / log(float(16))) + 1;

    for (int b = 0; b < TEST_BLOCK_COUNT; b++) {
        basic_erase_program_read_test(blockD, block_size, write_block, read_block, error_mask, addrwidth);
    }

    err = blockD.deinit();
    TEST_ASSERT_EQUAL(0, err);
}

void test_qspif_unaligned_program()
{

    printf("\nTest Unaligned Program Starts..\n");

    QSPIFBlockDevice blockD(MBED_CONF_QSPIF_QSPI_IO0, MBED_CONF_QSPIF_QSPI_IO1, MBED_CONF_QSPIF_QSPI_IO2,
                            MBED_CONF_QSPIF_QSPI_IO3, MBED_CONF_QSPIF_QSPI_CLK, MBED_CONF_QSPIF_QSPI_CS, 0, MBED_CONF_QSPIF_QSPI_FREQ);

    int err = blockD.init();
    TEST_ASSERT_EQUAL(0, err);

    for (unsigned atr = 0; atr < sizeof(ATTRS) / sizeof(ATTRS[0]); atr++) {
        static const char *prefixes[] = {"", "k", "M", "G"};
        for (int i_ind = 3; i_ind >= 0; i_ind--) {
            bd_size_t size = (blockD.*ATTRS[atr].method)();
            if (size >= (1ULL << 10 * i_ind)) {
                printf("%s: %llu%sbytes (%llubytes)\n",
                       ATTRS[atr].name, size >> 10 * i_ind, prefixes[i_ind], size);
                break;
            }
        }
    }

    bd_size_t block_size = blockD.get_erase_size();
    uint8_t *write_block = new uint8_t[block_size];
    uint8_t *read_block = new uint8_t[block_size];
    uint8_t *error_mask = new uint8_t[TEST_ERROR_MASK];
    unsigned addrwidth = ceil(log(float(blockD.size() - 1)) / log(float(16))) + 1;


    bd_addr_t block = (rand() * block_size) % blockD.size() + 15;

    // Use next random number as temporary seed to keep
    // the address progressing in the pseudorandom sequence
    unsigned seed = rand();

    // Fill with random sequence
    srand(seed);
    for (bd_size_t i_ind = 0; i_ind < block_size; i_ind++) {
        write_block[i_ind] = 0xff & rand();
    }

    // Write, sync, and read the block
    printf("\ntest  %0*llx:%llu...", addrwidth, block, block_size);

    err = blockD.erase(block, block_size);
    TEST_ASSERT_EQUAL(0, err);

    err = blockD.program(write_block, block, block_size);
    TEST_ASSERT_EQUAL(0, err);

    printf("\nwrite %0*llx:%llu ", addrwidth, block, block_size);
    for (int i_ind = 0; i_ind < 15; i_ind++) {
        printf("%02x", write_block[block_size - 16 + i_ind]);
    }
    printf("...\n");

    err = blockD.read(read_block, block, block_size);
    TEST_ASSERT_EQUAL(0, err);

    printf("read  %0*llx:%llu ", addrwidth, block, block_size);
    for (int i_ind = 0; i_ind < 15; i_ind++) {
        printf("%02x", read_block[block_size - 16 + i_ind]);
    }
    printf("...\n");

    // Find error mask for debugging
    memset(error_mask, 0, TEST_ERROR_MASK);
    bd_size_t error_scale = block_size / (TEST_ERROR_MASK * 8);

    srand(seed);
    for (bd_size_t i_ind = 0; i_ind < TEST_ERROR_MASK * 8; i_ind++) {
        for (bd_size_t j_ind = 0; j_ind < error_scale; j_ind++) {
            if ((0xff & rand()) != read_block[i_ind * error_scale + j_ind]) {
                error_mask[i_ind / 8] |= 1 << (i_ind % 8);
            }
        }
    }

    printf("error %0*llx:%llu ", addrwidth, block, block_size);
    for (int i_ind = 0; i_ind < 16; i_ind++) {
        printf("%02x", error_mask[i_ind]);
    }
    printf("\n");

    // Check that the data was unmodified
    srand(seed);
    for (bd_size_t i_ind = 0; i_ind < block_size; i_ind++) {
        TEST_ASSERT_EQUAL(0xff & rand(), read_block[i_ind]);
    }

    err = blockD.deinit();
    TEST_ASSERT_EQUAL(0, err);
}



static void test_qspif_thread_job(void *vBlockD/*, int thread_num*/)
{
    static int thread_num = 0;
    thread_num++;
    QSPIFBlockDevice *blockD = (QSPIFBlockDevice *)vBlockD;
    printf("\n Thread %d Started \n", thread_num);

    bd_size_t block_size = blockD->get_erase_size();
    uint8_t *write_block = new uint8_t[block_size];
    uint8_t *read_block = new uint8_t[block_size];
    uint8_t *error_mask = new uint8_t[TEST_ERROR_MASK];
    unsigned addrwidth = ceil(log(float(blockD->size() - 1)) / log(float(16))) + 1;

    for (int b = 0; b < TEST_BLOCK_COUNT; b++) {
        basic_erase_program_read_test((*blockD), block_size, write_block, read_block, error_mask, addrwidth);
    }


}

void test_qspif_multi_threads()
{

    printf("\nTest Multi Threaded Erase/Program/Read Starts..\n");

    QSPIFBlockDevice blockD(MBED_CONF_QSPIF_QSPI_IO0, MBED_CONF_QSPIF_QSPI_IO1, MBED_CONF_QSPIF_QSPI_IO2,
                            MBED_CONF_QSPIF_QSPI_IO3, MBED_CONF_QSPIF_QSPI_CLK, MBED_CONF_QSPIF_QSPI_CS, 0, MBED_CONF_QSPIF_QSPI_FREQ);

    int err = blockD.init();
    TEST_ASSERT_EQUAL(0, err);

    for (unsigned atr = 0; atr < sizeof(ATTRS) / sizeof(ATTRS[0]); atr++) {
        static const char *prefixes[] = {"", "k", "M", "G"};
        for (int i_ind = 3; i_ind >= 0; i_ind--) {
            bd_size_t size = (blockD.*ATTRS[atr].method)();
            if (size >= (1ULL << 10 * i_ind)) {
                printf("%s: %llu%sbytes (%llubytes)\n",
                       ATTRS[atr].name, size >> 10 * i_ind, prefixes[i_ind], size);
                break;
            }
        }
    }

    rtos::Thread first_qspif_bd_thread;
    rtos::Thread second_qspif_bd_thread;
    rtos::Thread third_qspif_bd_thread;

    osStatus threadStatus = first_qspif_bd_thread.start(test_qspif_thread_job, (void *)&blockD);
    if (threadStatus != 0) {
        printf("\n First Thread Start Failed!");
    }
    threadStatus = second_qspif_bd_thread.start(test_qspif_thread_job, (void *)&blockD);
    if (threadStatus != 0) {
        printf("\n Second Thread Start Failed!");
    }
    threadStatus = third_qspif_bd_thread.start(test_qspif_thread_job, (void *)&blockD);
    if (threadStatus != 0) {
        printf("\n Second Thread Start Failed!");
    }

    first_qspif_bd_thread.join();
    second_qspif_bd_thread.join();
    third_qspif_bd_thread.join();

    err = blockD.deinit();
    TEST_ASSERT_EQUAL(0, err);
}




// Test setup
utest::v1::status_t test_setup(const size_t number_of_cases)
{
    GREENTEA_SETUP(60, "default_auto");
    return verbose_test_setup_handler(number_of_cases);
}

Case cases[] = {
    Case("Testing unaligned program blocks", test_qspif_unaligned_program),
    Case("Testing read write random blocks", test_qspif_random_program_read_erase),
    Case("Testing Multi Threads Erase Program Read", test_qspif_multi_threads)
};

Specification specification(test_setup, cases);


int main()
{
    printf("MAIN STARTS\n");
    return !Harness::run(specification);
}
