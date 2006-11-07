SRCS_TEX= PatternGenerator.tex PatternGeneratorMain.tex 

PatternGeneratorMain.pdf: PatternGenerator.tex PatternGeneratorMain.tex 
	pdflatex PatternGeneratorMain
	bibtex PatternGeneratorMain
	pdflatex PatternGeneratorMain
	pdflatex PatternGeneratorMain

all: PatternGeneratorMain.pdf dox


PGM.tgz:
	cd ..;tar zcvf PGM.tgz ./doc/*.tex ./doc/Makefile \
	./doc/figures ./doc/pgbib.bib ./doc/isolatin1.sty ./doc/IEEEbib.bst
dox:
	doxygen pgjrl.dox	

clean:
	rm -f PatternGeneratorMain.aux PatternGenerator.aux PatternGeneratorMain.bbl PatternGeneratorMain.blg PatternGeneratorMain.log PatternGeneratorMain.toc PatternGeneratorMain.pdf *~
