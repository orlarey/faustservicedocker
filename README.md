# faustservicecloud

Build a Docker image for faustservice, the remote Faust compiler

- update `faust/` and `faustservice/` subdirectories with a `git pull`
- build the docker image with `make`
- test the docker image with `make test` and in a terminal in cloud-test run `./testall localhost`. All targets should return OK
- push the image on google repository using `make push`
- go the cloud console, click on the running faust-service-n and choose `create similar`
- wait a long time (several minutes) before faust-service-n+1 is operational
- maybe test it with `./testall ipaddress`
- then go to VPC network/External IP addresses and change the faust-service-n external IP in order to attribute it to faust-service-n+1
  

