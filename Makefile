build: 
	docker build -t grame/faustservicecloud:latest .

test:
	docker run -it -v /var/run/docker.sock:/var/run/docker.sock -v /tmp/sessions:/tmp/sessions -p 80:80 grame/faustservicecloud:latest

debug:
	docker run -it -v /var/run/docker.sock:/var/run/docker.sock -v /tmp/sessions:/tmp/sessions -p 80:80 grame/faustservicecloud:latest /bin/bash

update: initsubmodules updatefaust updatefaustservice updatecrossosx

initsubmodules:
	git submodule update --init --recursive

updatefaust:
	git -C faust checkout master-dev
	git -C faust pull
	
updatefaustservice:
	git -C faustservice checkout server
	git -C faustservice pull

updatecrossosx:
	git -C faustcrossosx checkout master
	git -C faustcrossosx pull
	cd faustcrossosx && sh prepare.sh

help:
	@echo " 'update' : update faust and faustservice to the latest versions"
	@echo " 'build'  : builds the docker image"
	@echo " 'test'   : run the docker image"
	@echo " 'osxtest': run the docker image"
	@echo " 'debug'  : run the docker image in bash mode"

SHARED   := $(shell pwd)
osxtest:
	docker run -t -w /osxcross/tests -v $(SHARED):/osxcross/shared grame/faustservicecloud:latest ./run.sh -dest /osxcross/shared
	tar xzf MacOSX-Cross.tgz
	rm MacOSX-Cross.tgz
	@echo "Cross compiled files are available from the MacOSX-Cross folder."

