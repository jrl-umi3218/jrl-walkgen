walkGenJRLDocdir = $(datadir)/doc/walkGenJRL

install:html
	mkdir -p $(walkGenJRLDocdir)
	$(INSTALL_DATA) $</* $(walkGenJRLDocdir)

html: pgjrl.dox
	doxygen $<
