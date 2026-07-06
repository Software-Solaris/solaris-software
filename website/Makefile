MD2HTML = build/md2html/md2html
MDFLAGS = --github

SRCS   := $(wildcard src/*.md)
PAGES  := $(patsubst src/%.md,output/%.html,$(SRCS))

all: $(PAGES) output/style.css output/assets

# cada .md -> .html con menubar + footer
output/%.html: src/%.md template/header.html template/footer.html | output
	cat template/header.html  >  $@
	$(MD2HTML) $(MDFLAGS) $<   >> $@
	cat template/footer.html  >> $@

# copia el css que tienes en src/
output/style.css: src/style.css | output
	cp $< $@

# copia la carpeta assets (fuente, favicon, logo...) a output/assets/
output/assets: assets | output
	cp -r assets output/assets

output:
	mkdir -p output

clean:
	rm -rf output

.PHONY: all clean