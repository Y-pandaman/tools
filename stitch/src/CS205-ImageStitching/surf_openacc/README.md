## OpenACC Version
### Compile the Code
The OpenACC version can only be compiled and run on a device with GPU. Before compiling, set the environment with `source env.sh`.

#### On machine with GTX GPU, compile with: 

```
pgc++ -acc -ta=tesla:cc60 -Minfo -std=c++11 -O3 -o test main.cpp fasthessian.cpp integral.cpp ipoint.cpp surf.cpp utils.cpp `pkg-config opencv --cflags --libs`
```

#### On machine with Tesla GPU, compile with: 

```
pgc++ -acc -ta=tesla -Minfo -std=c++11 -O3 -o test main.cpp fasthessian.cpp integral.cpp ipoint.cpp surf.cpp utils.cpp `pkg-config opencv --cflags --libs`
```

### Run the Code

``./test -m <mode> [...]``

**Argument Options** (< > after flag indicates argument is required)

- -m | --mode < >: 

	- 0: run SURF on a single image
	- 1: run static image match between a pair of images
	- 2: run image stitching with webcam stream
        - 3: run image stitching with local video files

- -b | --blend_mode: (no additional argument)
         
	- if set, run blending algorithm with stitching; otherwise, run regular stitching algorithm

- -r | --resolution < >:

	- user-specified resolution

- -s | --single\_mem\_cpy: (OpenACC only)

	- if set, one single mem copy; otherwise, do memory copy from host to device every response layer

- -t | --threaded: (OpenACC only)

	- if set, using the multi-threading version for task-level parallelization

- -S/L/R | --src/src1/src2 <path>
         
	- <path> path of  image/video to be processed. For mode 0, `-S|--src` will be used for single image feature extraction; for mode 1 and mode 3, `-LR|--src1 --src2` will be used for image/video stitching from local files

	- if flags are not set, will use sample image/video given by this repository

### Test Case
Run:

``sh sample_test.sh``

The test shell script will run our simple test through our simplest image stitching option, pop up the live view of stitched image. Pressing ``Esc`` button will exit the live view and save the image data to a text file 'stitched.txt'.

The test shell script will check the output with the standard output running on our development machine. If it does not print anything after "Took: xxx seconds" line, then you are good to go!