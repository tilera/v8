export PATH=/home/jiwang/depot_tools:$PATH
make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug.check regexp=interpreted -j40 2>&1 | tee check.log
