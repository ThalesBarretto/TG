TEXS	=$(wildcard *.tex)
TEXC	=pdflatex -shell-escape -synctex=1 -interaction=nonstop -file-line-error-style

pdf: main.tex
	$(TEXC) $<
	$(TEXC) $<

clean:
	find . -name '*.aux' -or -name '*.log' -or -name '*.lot' -or -name '*.lof' -or -name '*.toc' \( -exec rm '{}' \; \)

all: pdf

.PHONY: pdf all
