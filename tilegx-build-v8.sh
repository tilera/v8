#make native -j40 2>&1 | tee build.log
#make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug snapshot=off -j40 2>&1 | tee build.log
#make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug -j40 2>&1 | tee build.log
#./out/tilegx.debug/shell
#make tilegx.debug -j40 2>&1 | tee build.log
#make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug.check regexp=interpreted snapshot=off -j40 2>&1 | tee build.log
#make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug regexp=interpreted -j40 2>&1
#make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug snapshot=off regexp=interpreted -j40 2>&1
make LDFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" CXXFLAGS+="-B/home/jiwang/GNU/build-tilegx-binutils/gold" tilegx.debug regexp=interpreted -j40 2>&1
