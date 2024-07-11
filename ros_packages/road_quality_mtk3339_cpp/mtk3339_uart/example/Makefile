CC := g++
CXXFLAGS := -O3
LDFLAGS := -lnmea -lyaml-cpp

all: test test_yaml

mtk_serial.o:
	$(CC) $(CXXFLAGS) -c ../mtk_serial.cpp $(LDFLAGS)

mtk_utils.o:
	$(CC) $(CXXFLAGS) -c ../mtk_utils.cpp $(LDFLAGS)

test: mtk_serial.o mtk_utils.o
	$(CC) $(CXXFLAGS) test.cpp mtk_serial.o mtk_utils.o -I ../ -o test $(LDFLAGS)

test_yaml: mtk_utils.o
	$(CC) $(CXXFLAGS) test_yaml.cpp mtk_utils.o -I ../ -o test_yaml $(LDFLAGS)

.PHONY: clean
clean:
	@rm -f *.o
	@rm -f test
	@rm -f test_yaml