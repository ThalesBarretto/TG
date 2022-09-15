TEXC	=pdflatex
TEXCFLAG=-shell-escape \
	-synctex=1 \
	-interaction=nonstopmode \
	-file-line-error

BIBC	=bibtex
BIBCFLAG=

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
pdf:
	@-$(TEXC) $(TEXCFLAG) $(TEXMAIN) >>/dev/null 2>&1
	@$(BIBC) $(BIBCFLAG) $(patsubst %.tex,%.aux,$(TEXMAIN))
	@-$(TEXC) $(TEXCFLAG) $(TEXMAIN) >>/dev/null 2>&1
	$(TEXC) $(TEXCFLAG) $(TEXMAIN)

show:
	-$(PDFVIEW) $(PDFS) & disown

clean:
	-latexmk -c --quiet >/dev/null 2>&1

wipe:
	-latexmk -C --quiet >/dev/null 2>&1
	-rm -f *.blg *.bbl *.brf *.log

tidy: clean | pdf

all: pdf show

.PHONY: pdf all clean wipe
