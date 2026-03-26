SDK_DIR = third_party/Livox-SDK2
SDK_BUILD_DIR = $(SDK_DIR)/build
SDK_STATIC_LIB = $(SDK_BUILD_DIR)/sdk_core/liblivox_lidar_sdk_static.a

.PHONY: all sdk clean

all: sdk

sdk: $(SDK_STATIC_LIB)

$(SDK_STATIC_LIB):
	# Patch out -Werror for Apple clang compatibility (build-time only, not committed)
	cd $(SDK_DIR) && sed -i.bak 's/-Werror //g' sdk_core/CMakeLists.txt
	mkdir -p $(SDK_BUILD_DIR)
	cd $(SDK_BUILD_DIR) && cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
	cd $(SDK_BUILD_DIR) && make -j$$(sysctl -n hw.ncpu 2>/dev/null || nproc)
	# Restore original CMakeLists.txt
	cd $(SDK_DIR) && mv sdk_core/CMakeLists.txt.bak sdk_core/CMakeLists.txt

clean:
	rm -rf $(SDK_BUILD_DIR)
