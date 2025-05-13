import platform
import psutil
import os
from cpuinfo import get_cpu_info
import GPUtil
try:
    import pynvml
except ImportError:
    pynvml = None
try:
    import pyopencl as cl
except ImportError:
    cl = None
try:
    import pycuda.driver as drv
    import pycuda.autoinit  # This initializes CUDA
except ImportError:
    drv = None

ASSETS_DIR = "./assets"

def get_nvidia_cores_per_sm(compute_capability):
    """Returns cores per SM for NVIDIA GPUs based on compute capability."""
    # Lookup table for cores per SM (FP32) by architecture
    # Based on NVIDIA documentation
    cores_per_sm = {
        (3, 0): 192,  # Kepler
        (3, 5): 192,  # Kepler
        (5, 0): 128,  # Maxwell
        (5, 2): 128,  # Maxwell
        (6, 0): 64,   # Pascal (e.g., GTX 1080)
        (6, 1): 64,   # Pascal
        (6, 2): 64,   # Pascal
        (7, 0): 64,   # Volta
        (7, 5): 64,   # Turing (e.g., RTX 2080)
        (8, 0): 64,   # Ampere (e.g., A100)
        (8, 6): 64,   # Ampere (e.g., RTX 3080)
        (8, 9): 64,   # Ada Lovelace (e.g., RTX 4090)
        (9, 0): 64,   # Hopper
    }
    return cores_per_sm.get(compute_capability, 64)  # Default to 64 if unknown

def get_cuda_device_theoretical_tflops():
    if not drv:
        print("PyCUDA not installed. Cannot calculate NVIDIA GPU TFLOPS.")
        return None
    try:
        device = drv.Device(0)  # Get the first CUDA device
        attrs = device.get_attributes()

        # Key attributes for TFLOPS calculation (FP32)
        num_sms = attrs[drv.device_attribute.MULTIPROCESSOR_COUNT]
        clock_khz = attrs[drv.device_attribute.CLOCK_RATE]
        clock_ghz = clock_khz / 1_000_000.0

        # Get compute capability
        compute_capability = device.compute_capability()  # Returns tuple (major, minor)
        cores_per_sm = get_nvidia_cores_per_sm(compute_capability)
        ops_per_core_per_clock_fp32 = 2  # FP32 FMA (2 FLOPs per cycle)

        total_cores = num_sms * cores_per_sm

        # TFLOPS calculation (FP32)
        tflops = (total_cores * clock_ghz * ops_per_core_per_clock_fp32) / 1_000

        # print(f"GPU Name: {device.name()}")
        # print(f"Compute Capability: {compute_capability[0]}.{compute_capability[1]}")
        # print(f"Number of SMs: {num_sms}")
        # print(f"Cores per SM: {cores_per_sm}")
        # print(f"Total Cores: {total_cores}")
        # print(f"Max Clock Rate: {clock_ghz:.2f} GHz")
        # print(f"Theoretical Peak FP32 TFLOPS: {tflops:.2f}")

        return tflops

    except drv.Error as e:
        print(f"Error getting CUDA device info: {e}")
        print("Please ensure CUDA is installed and configured correctly.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

def get_system_info():
    """Retrieves system information including CPU, RAM, and GPU (model, VRAM, calculated TFLOPS)."""
    try:
        # CPU Information
        if get_cpu_info:
            cpu_info_dict = get_cpu_info()
            cpu_model = cpu_info_dict.get('brand_raw', 'Unknown CPU')
        else:
            cpu_model = platform.processor() or 'Unknown CPU'
            print("Warning: 'cpuinfo' not installed. CPU model may be inaccurate.")
        
        cpu_cores = psutil.cpu_count(logical=False) or "Unknown"
        cpu_threads = psutil.cpu_count(logical=True) or "Unknown"
        cpu_info = (
            f"CPU Model: {cpu_model}"
        )

        # RAM Information
        ram = psutil.virtual_memory()
        ram_total = ram.total / (1024 ** 3)  # Convert to GB
        ram_used = ram.used / (1024 ** 3)    # Convert to GB
        ram_info = (
            f"Total RAM: {ram_total:.2f} GB"
        )

        # GPU Information
        if not GPUtil:
            gpu_info = "GPU Info: GPUtil not installed"
        else:
            try:
                gpus = GPUtil.getGPUs()
                if not gpus:
                    gpu_info = "GPU Info: No compatible GPUs detected"
                else:
                    gpu_info = []
                    reference_tflops = 103.0  # RTX 5090 reference

                    # Initialize pynvml if available
                    if pynvml:
                        pynvml.nvmlInit()

                    for i, gpu in enumerate(gpus):
                        tflops_str = "Unknown"
                        vram = gpu.memoryTotal / 1024  # Convert MB to GB

                        # NVIDIA GPU
                        if "nvidia" in gpu.name.lower():
                            tflops = get_cuda_device_theoretical_tflops()
                            tflops_str = f"{tflops:.1f}" if tflops else "Unknown"

                        # AMD GPU
                        elif "amd" in gpu.name.lower() or "radeon" in gpu.name.lower():
                            if not cl:
                                tflops_str = "PyOpenCL not installed"
                            else:
                                try:
                                    platforms = cl.get_platforms()
                                    for pl in platforms:
                                        if "amd" in pl.name.lower():
                                            devices = pl.get_devices(device_type=cl.device_type.GPU)
                                            for device in devices:
                                                if gpu.name.lower() in device.name.lower():
                                                    compute_units = device.max_compute_units
                                                    clock_mhz = device.max_clock_frequency
                                                    clock_ghz = clock_mhz / 1000.0
                                                    cores_per_cu = 64  # RDNA architecture (approximate)
                                                    total_cores = compute_units * cores_per_cu
                                                    tflops = (total_cores * clock_ghz * 2) / 1000  # 2 FLOPs per cycle
                                                    tflops_str = f"{tflops:.1f}"
                                                    break
                                            else:
                                                continue
                                            break
                                    else:
                                        tflops_str = "No matching AMD GPU found"
                                except cl.CLException as e:
                                    tflops_str = f"Error calculating TFLOPS: {str(e)}"

                        # Other GPUs (e.g., Intel)
                        else:
                            tflops_str = "Unsupported GPU (e.g., Intel Arc requires additional support)"

                        gpu_info.append(
                            f"GPU {i}: {gpu.name}, "
                            f"TFLOPS: {tflops_str}"
                        )

                    # Cleanup pynvml
                    if pynvml:
                        pynvml.nvmlShutdown()

                    gpu_info = "\n".join(gpu_info)

            except Exception as e:
                gpu_info = f"GPU Info: Error accessing GPU - {str(e)} (ensure GPU drivers are installed)"

        # Combine all info
        return f"{cpu_info}\n{ram_info}\n{gpu_info}"

    except Exception as e:
        return f"Error retrieving system info: {str(e)}"

if __name__ == "__main__":
    print(get_system_info())




# Estimator
def estimate_generation_time(generation_mode, asset_folder_name, parts_to_generate):
    """Placeholder function to estimate generation time."""
    lookup_basetimes_with_baseline_hardware = {
        "Production": {
            "Face Only": 4512,
            "Hair Only": 7715
        }, "Prototyping": {
            "Face Only": 496,
            "Hair Only": 319
        }
    }
    lookup_basetimes_with_baseline_hardware["Production"]["Face & Hair"] = lookup_basetimes_with_baseline_hardware["Production"]["Face Only"] + lookup_basetimes_with_baseline_hardware["Production"]["Hair Only"]
    lookup_basetimes_with_baseline_hardware["Prototyping"]["Face & Hair"] = lookup_basetimes_with_baseline_hardware["Prototyping"]["Face Only"] + lookup_basetimes_with_baseline_hardware["Prototyping"]["Hair Only"]
    base_time = lookup_basetimes_with_baseline_hardware.get(generation_mode, {}).get(parts_to_generate, 0)

    baseline_theoretical_tflops = 19.49184  # Example baseline TFLOPS for a GTX 1080

    # To estimate the time, we compare the theoretical TFLOPS of the current GPU to the baseline
    current_tflops = get_cuda_device_theoretical_tflops()
    estimated_seconds = base_time / (current_tflops / baseline_theoretical_tflops)

    def format_time(estimated_seconds):
        if estimated_seconds < 60:
            text = f"{int(estimated_seconds)}s"
        elif estimated_seconds < 3600:
            minutes = int(estimated_seconds // 60)
            seconds = int(estimated_seconds % 60)
            text = f"{minutes}m{seconds}s"
        else:
            hours = int(estimated_seconds // 3600)
            remaining_seconds = estimated_seconds % 3600
            minutes = int(remaining_seconds // 60)
            seconds = int(remaining_seconds % 60)
            text = f"{hours}h{minutes}m{seconds}s"
        return text

    return format_time(estimated_seconds)