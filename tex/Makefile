BIBSRCS	=base-referencias.bib
TEXSRC	=. configuracoes figuras pos-textuais pre-textuais textuais
TEXSRCS	=$(foreach var,$(TEXSRC),$(wildcard $(var)/*.tex))
TEXMAIN	=main.tex
TEXPDFS	=$(patsubst %.tex, %.pdf, $(findstring $(TEXMAIN), $(TEXSRCS)))

# PDF files to be generated in the whole project
PDFS	=$(TEXPDFS)
# System PDF viewer
PDFVIEW	=okular

# Don't use file recipes, use rules and let pdflatex handle this
pdf: $(TEXMAIN)
	-@latexmk --quiet $< >/dev/null 2>&1

show:
	-@$(shell nohup $(PDFVIEW) $(PDFS) > /dev/null 2>&1 > /dev/null &!)

clean:
	-@latexmk -c --quiet >/dev/null 2>&1

wipe:
	-@latexmk -C --quiet >/dev/null 2>&1

tidy: pdf clean

all: pdf

.PHONY: show pdf all clean wipe
