#!/bin/bash
sh build.sh
cd bin
./CodeCraft-2019 ../config/car.txt ../config/road.txt ../config/cross.txt ../config/presetAnswer.txt ../config/answer.txt
cd ../CodeCraft-2019/judge/
make
make clean
@echo Testing schdule judge...
./main ../../training-answer/car.txt ../../training-answer/road.txt ../../training-answer/cross.txt ../../training-answer/presetAnswer.txt ../../training-answer/answer.txt
