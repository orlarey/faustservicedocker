FAUSTSERVICEDOCKER=eu.gcr.io/faust-cloud-208407/faustservicecloud
VERSION=version20230906

image: 
	docker build -t $(FAUSTSERVICEDOCKER):$(VERSION) .

test:
	docker run -it -p 80:80 $(FAUSTSERVICEDOCKER):$(VERSION)

push:
	docker push $(FAUSTSERVICEDOCKER):$(VERSION)

debug:
	docker run -it -p 80:80 $(FAUSTSERVICEDOCKER):$(VERSION) /bin/bash 

update: initsubmodules updatecrossosx

initsubmodules:
	git submodule update --init --recursive

updatecrossosx:
	git -C faustcrossosx checkout master
	git -C faustcrossosx pull
	cd faustcrossosx && sh prepare.sh

help:
	@echo " 'update' : update faust and faustservice to the latest versions"
	@echo " 'image'  : builds the docker image"
	@echo " 'test'   : run the docker image"
	@echo " 'osxtest': run the docker image"
	@echo " 'debug'  : run the docker image in bash mode"
	@echo " 'push' 	 : push the docker image to docker repository"

SHARED   := $(shell pwd)
osxtest:
	docker run -t -w /osxcross/tests -v $(SHARED):/osxcross/shared $(FAUSTSERVICEDOCKER):$(VERSION) ./run.sh -dest /osxcross/shared
	tar xzf MacOSX-Cross.tgz
	rm MacOSX-Cross.tgz
	@echo "Cross compiled files are available from the MacOSX-Cross folder."

updateimage:
	docker commit --change='CMD ["./faustweb", "--port", "80", "--sessions-dir", "/tmp/sessions", "--recover-cmd", "/faustservice/faustweb"]' 488dc624a2c0 eu.gcr.io/faust-cloud-208407/faustservicecloud:version20230605
