UNAME_S := $(shell uname -s 2>/dev/null || echo Unknown)
ifeq ($(UNAME_S),Darwin)
	NPROC := $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
else
	NPROC := $(shell nproc --all 2>/dev/null || echo 4)
endif

.PHONY: pyinstall pyinstall_with_demo cppinstall cppinstall_with_demo

pyinstall:
	@python3 -m pip install --upgrade pip
	@pip install --verbose ./python/

pyinstall_with_demo: pyinstall
	@pip install open3d==0.18.0

cppinstall:
	@cmake -Bcpp/build cpp/
	@cmake --build cpp/build -j$(NPROC)

cppinstall_with_demo:
	@cmake -Bcpp/build cpp/ -DINCLUDE_CPP_EXAMPLES=ON
	@cmake --build cpp/build -j$(NPROC)
