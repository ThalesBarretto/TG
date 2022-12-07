TEXMAIN	=main.tex

all:  $(TEXMAIN)
	latexmk $<

show: $(TEXMAIN)
	latexmk --quiet -view=pdf -pv $<

clean:
	-@latexmk -c --quiet >/dev/null 2>&1

wipe:
	-@latexmk -C --quiet >/dev/null 2>&1

.DEFAULT_GOAL := all
.PHONY: pdf show run
