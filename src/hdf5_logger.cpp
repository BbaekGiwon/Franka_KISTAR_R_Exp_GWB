#include "hdf5_logger.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/time.h>
#include <vector>

#include <H5Cpp.h>
#include <H5Dpublic.h>
#include <H5Fpublic.h>
#include <H5Ipublic.h>
#include <H5Ppublic.h>
#include <H5Spublic.h>
#include <H5Tpublic.h>

namespace
{
constexpr hsize_t kChunkRows = 256;

std::string make_log_file_path()
{
    const std::string log_dir = std::string(KISTAR_PROJECT_ROOT) + "/log";
    if (mkdir(log_dir.c_str(), 0777) != 0 && errno != EEXIST)
    {
        throw std::runtime_error("Failed to create log directory");
    }

    timeval tv{};
    gettimeofday(&tv, nullptr);

    tm tm_now{};
    localtime_r(&tv.tv_sec, &tm_now);

    char ts[64] = {0};
    std::snprintf(ts,
                  sizeof(ts),
                  "%04d%02d%02d_%02d%02d%02d_%03ld",
                  tm_now.tm_year + 1900,
                  tm_now.tm_mon + 1,
                  tm_now.tm_mday,
                  tm_now.tm_hour,
                  tm_now.tm_min,
                  tm_now.tm_sec,
                  tv.tv_usec / 1000);

    return log_dir + "/hand_log_" + ts + ".hdf5";
}

// C API로 생성해 ID만 반환 (C++ 래퍼 임시 객체로 인한 ID 무효화 방지)
hid_t create_unlimited_dataset_c(hid_t file_id,
                                const char *name,
                                hid_t dtype_id,
                                const std::vector<hsize_t> &tail_dims)
{
    const size_t rank = tail_dims.size() + 1;
    std::vector<hsize_t> dims(rank, 0);
    std::vector<hsize_t> max_dims(rank, H5S_UNLIMITED);
    std::vector<hsize_t> chunk(rank, 1);

    for (size_t i = 0; i < tail_dims.size(); ++i)
    {
        dims[i + 1] = tail_dims[i];
        max_dims[i + 1] = tail_dims[i];
        chunk[i + 1] = tail_dims[i];
    }
    chunk[0] = kChunkRows;

    hid_t space_id = H5Screate_simple(static_cast<int>(rank), dims.data(), max_dims.data());
    if (space_id < 0)
        return -1;
    hid_t plist_id = H5Pcreate(H5P_DATASET_CREATE);
    if (plist_id < 0)
    {
        H5Sclose(space_id);
        return -1;
    }
    if (H5Pset_chunk(plist_id, static_cast<int>(rank), chunk.data()) < 0)
    {
        H5Pclose(plist_id);
        H5Sclose(space_id);
        return -1;
    }
    hid_t dset_id = H5Dcreate2(file_id, name, dtype_id, space_id, H5P_DEFAULT, plist_id, H5P_DEFAULT);
    H5Pclose(plist_id);
    H5Sclose(space_id);
    if (dset_id < 0 || H5Iis_valid(dset_id) <= 0)
        return H5I_INVALID_HID;
    return dset_id;
}

void append_dataset_c(hid_t dset_id,
                     hid_t dtype_id,
                     const void *data,
                     const std::vector<hsize_t> &tail_dims,
                     hsize_t row)
{
    if (dset_id < 0 || H5Iis_valid(dset_id) <= 0)
        return;
    const size_t rank = tail_dims.size() + 1;
    std::vector<hsize_t> new_dims(rank, 1);
    hsize_t start[8];
    hsize_t count[8];
    for (size_t i = 0; i < rank; i++)
        start[i] = 0, count[i] = 1;

    new_dims[0] = row + 1;
    for (size_t i = 0; i < tail_dims.size(); ++i)
    {
        new_dims[i + 1] = tail_dims[i];
        count[i + 1] = tail_dims[i];
    }

    if (H5Dset_extent(dset_id, new_dims.data()) < 0)
        return;

    hid_t file_space_id = H5Dget_space(dset_id);
    start[0] = row;
    H5Sselect_hyperslab(file_space_id, H5S_SELECT_SET, start, nullptr, count, nullptr);

    hid_t mem_space_id = H5Screate_simple(static_cast<int>(rank), count, nullptr);
    H5Dwrite(dset_id, dtype_id, mem_space_id, file_space_id, H5P_DEFAULT, data);
    H5Sclose(mem_space_id);
    H5Sclose(file_space_id);
}
} // namespace

class Hdf5HandLogger::Impl
{
public:
    Impl()
        : file_path(make_log_file_path()),
          file_id(H5Fcreate(file_path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT)),
          row_count(0),
          valid_(true)
    {
        if (file_id < 0 || H5Iis_valid(file_id) <= 0) {
            valid_ = false;
            return;
        }
        ds_timestamp_ns = create_unlimited_dataset_c(file_id, "00_timestamp_ns", H5T_NATIVE_ULLONG, {});
        ds_j_pos = create_unlimited_dataset_c(file_id, "01_j_pos", H5T_NATIVE_SHORT, {Hand_Num, Hand_DOF});
        ds_j_tar = create_unlimited_dataset_c(file_id, "02_j_tar", H5T_NATIVE_SHORT, {Hand_Num, Hand_DOF});
        ds_j_cur = create_unlimited_dataset_c(file_id, "03_j_cur", H5T_NATIVE_SHORT, {Hand_Num, Hand_DOF});
        ds_j_kin = create_unlimited_dataset_c(file_id, "04_j_kin", H5T_NATIVE_SHORT,
                                              {Hand_Num, Kinesthetic_Sensor_Num, Kinesthetic_Sensor_DOF});
        ds_j_tac = create_unlimited_dataset_c(file_id, "05_j_tac", H5T_NATIVE_SHORT, {Hand_Num, Tactile_Sensor_Num});
        ds_tip_pos = create_unlimited_dataset_c(file_id, "06_tip_pos", H5T_NATIVE_FLOAT,
                                                {Hand_Num, Finger_Num, Cartesian_DOF});
        ds_tip_quat = create_unlimited_dataset_c(file_id, "07_tip_quat", H5T_NATIVE_FLOAT,
                                                 {Hand_Num, Finger_Num, Quaternion_DOF});
        ds_arm_j_pos = create_unlimited_dataset_c(file_id, "08_arm_j_pos", H5T_NATIVE_DOUBLE, {Arm_Num, Arm_DOF});
        ds_arm_j_tar = create_unlimited_dataset_c(file_id, "09_arm_j_tar", H5T_NATIVE_DOUBLE, {Arm_Num, Arm_DOF});
        ds_arm_j_vel = create_unlimited_dataset_c(file_id, "10_arm_j_vel", H5T_NATIVE_DOUBLE, {Arm_Num, Arm_DOF});
        ds_arm_c_pos = create_unlimited_dataset_c(file_id, "11_arm_c_pos", H5T_NATIVE_DOUBLE, {Arm_Num, 16});
        ds_arm_j_tq = create_unlimited_dataset_c(file_id, "12_arm_j_tq", H5T_NATIVE_DOUBLE, {Arm_Num, Arm_DOF});
        auto is_invalid = [](hid_t id) { return id < 0 || H5Iis_valid(id) <= 0; };
        if (is_invalid(ds_timestamp_ns) || is_invalid(ds_j_pos) || is_invalid(ds_j_tar) ||
            is_invalid(ds_j_cur) || is_invalid(ds_j_kin) || is_invalid(ds_j_tac) ||
            is_invalid(ds_tip_pos) || is_invalid(ds_tip_quat) || is_invalid(ds_arm_j_pos) ||
            is_invalid(ds_arm_j_tar) || is_invalid(ds_arm_j_vel) || is_invalid(ds_arm_c_pos) ||
            is_invalid(ds_arm_j_tq)) {
            valid_ = false;
        }
    }

    ~Impl()
    {
        auto close_if_valid = [](hid_t& id) {
            if (id >= 0 && H5Iis_valid(id) > 0) { H5Dclose(id); id = -1; }
        };
        close_if_valid(ds_timestamp_ns);
        close_if_valid(ds_j_pos);
        close_if_valid(ds_j_tar);
        close_if_valid(ds_j_cur);
        close_if_valid(ds_j_kin);
        close_if_valid(ds_j_tac);
        close_if_valid(ds_tip_pos);
        close_if_valid(ds_tip_quat);
        close_if_valid(ds_arm_j_pos);
        close_if_valid(ds_arm_j_tar);
        close_if_valid(ds_arm_j_vel);
        close_if_valid(ds_arm_c_pos);
        close_if_valid(ds_arm_j_tq);
        if (file_id >= 0 && H5Iis_valid(file_id) > 0) {
            H5Fclose(file_id);
            file_id = -1;
        }
    }

    void append(const SHMmsgs &sample)
    {
        if (!valid_) return;
        timeval tv{};
        gettimeofday(&tv, nullptr);
        const uint64_t now_ns = static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL +
                                static_cast<uint64_t>(tv.tv_usec) * 1000ULL;

        append_dataset_c(ds_j_pos, H5T_NATIVE_SHORT, sample.j_pos, {Hand_Num, Hand_DOF}, row_count);
        append_dataset_c(ds_j_tar, H5T_NATIVE_SHORT, sample.j_tar, {Hand_Num, Hand_DOF}, row_count);
        append_dataset_c(ds_j_cur, H5T_NATIVE_SHORT, sample.j_cur, {Hand_Num, Hand_DOF}, row_count);
        append_dataset_c(ds_j_kin, H5T_NATIVE_SHORT,
                        sample.j_kin, {Hand_Num, Kinesthetic_Sensor_Num, Kinesthetic_Sensor_DOF}, row_count);
        append_dataset_c(ds_j_tac, H5T_NATIVE_SHORT, sample.j_tac, {Hand_Num, Tactile_Sensor_Num}, row_count);
        append_dataset_c(ds_tip_pos, H5T_NATIVE_FLOAT,
                        sample.tip_pos, {Hand_Num, Finger_Num, Cartesian_DOF}, row_count);
        append_dataset_c(ds_tip_quat, H5T_NATIVE_FLOAT,
                        sample.tip_quat, {Hand_Num, Finger_Num, Quaternion_DOF}, row_count);
        append_dataset_c(ds_arm_j_pos, H5T_NATIVE_DOUBLE,
                        sample.Arm_j_pos, {Arm_Num, Arm_DOF}, row_count);
        append_dataset_c(ds_arm_j_tar, H5T_NATIVE_DOUBLE,
                        sample.Arm_j_tar, {Arm_Num, Arm_DOF}, row_count);
        append_dataset_c(ds_arm_j_vel, H5T_NATIVE_DOUBLE,
                        sample.Arm_j_vel, {Arm_Num, Arm_DOF}, row_count);
        append_dataset_c(ds_arm_c_pos, H5T_NATIVE_DOUBLE,
                        sample.Arm_C_Pos, {Arm_Num, 16}, row_count);
        append_dataset_c(ds_arm_j_tq, H5T_NATIVE_DOUBLE,
                        sample.Arm_j_tq, {Arm_Num, Arm_DOF}, row_count);
        append_dataset_c(ds_timestamp_ns, H5T_NATIVE_ULLONG, &now_ns, {}, row_count);

        row_count++;
    }

    std::string file_path;
    hid_t file_id = -1;

    hid_t ds_timestamp_ns = -1;
    hid_t ds_j_pos = -1;
    hid_t ds_j_tar = -1;
    hid_t ds_j_cur = -1;
    hid_t ds_j_kin = -1;
    hid_t ds_j_tac = -1;
    hid_t ds_tip_pos = -1;
    hid_t ds_tip_quat = -1;
    hid_t ds_arm_j_pos = -1;
    hid_t ds_arm_j_tar = -1;
    hid_t ds_arm_j_vel = -1;
    hid_t ds_arm_c_pos = -1;
    hid_t ds_arm_j_tq = -1;

    hsize_t row_count = 0;
    bool valid_ = false;
};

Hdf5HandLogger::Hdf5HandLogger()
    : impl_(nullptr)
{
    impl_ = new Impl();
}

Hdf5HandLogger::~Hdf5HandLogger()
{
    delete impl_;
    impl_ = nullptr;
}

bool Hdf5HandLogger::is_open() const
{
    return impl_ != nullptr;
}

const std::string &Hdf5HandLogger::file_path() const
{
    return impl_->file_path;
}

void Hdf5HandLogger::append(const SHMmsgs &sample)
{
    impl_->append(sample);
}
