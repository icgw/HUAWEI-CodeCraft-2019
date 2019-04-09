#!/bin/bash
make
./main ../../2-training-training-1-answer/car.txt ../../2-training-training-1-answer/road.txt ../../2-training-training-1-answer/cross.txt ../../2-training-training-1-answer/presetAnswer.txt ../../2-training-training-1-answer/answer.txt

./main ../../2-training-training-2-answer/car.txt ../../2-training-training-2-answer/road.txt ../../2-training-training-2-answer/cross.txt ../../2-training-training-2-answer/presetAnswer.txt ../../2-training-training-2-answer/answer.txt

# ./main ../../test_config/car.txt ../../test_config/road.txt ../../test_config/cross.txt ../../test_config/presetAnswer.txt ../../test_config/answer.txt
./main ../../2-map-training-1/car.txt ../../2-map-training-1/road.txt ../../2-map-training-1/cross.txt ../../2-map-training-1/presetAnswer.txt ../../2-map-training-1/answer.txt
