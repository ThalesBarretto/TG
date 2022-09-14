TEXS	=$(wildcard *.tex)
TEXC	=pdflatex -shell-escape -synctex=1 -interaction=nonstop -file-line-error-style

pdf: main.tex
	$(TEXC) $<

.PHONY: pdf
