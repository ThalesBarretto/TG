TEX	=tex

tex:
	cd $(TEX) && make

all: tex

.PHONY: all tex
