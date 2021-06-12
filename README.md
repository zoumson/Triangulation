[![LinkedIn][linkedin-shield]][linkedin-url]
<!--
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]


[![Github][github-shield]][github.com/zoumson?tab=repositories]
[![Stack Overflow][stackoverflow-shield]][stackoverflow.com/users/11175375/adam]
[![Leetcode][leetcode-shield]][eetcode.com/Hard_Code/]
-->
## Triangulation 

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#file-structure">Files Structure</a>
      <ul>
        <li><a href="#folders">Folders</a></li>
        <li><a href="#entire-files-structure">Entire Files Structure</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

Computer stereo vision and optical 3D measuring systems use this principle to determine the spatial dimensions and the geometry of an item.[2] Basically, the configuration consists of two sensors observing the item. One of the sensors is typically a digital camera device, and the other one can also be a camera or a light projector. The projection centers of the sensors and the considered point on the object's surface define a (spatial) triangle. Within this triangle, the distance between the sensors is the base b and must be known. By determining the angles between the projection rays of the sensors and the basis, the intersection point, and thus the 3D coordinate, is calculated from the triangular relations.

<!--Built with -->
### Built With

<br>

* [cmake](https://cmake.org/)
* [gnu](https://www.gnu.org/)
* [g2o](https://github.com/RainerKuemmerle/g2o)
* [opencv](https://opencv.org/)
<br>

## File Structure

### Folders

* [include/](include/): c++ header files.
* [ressource/](ressource/): image files.
* [src/](src/): c++ definitions.


### Entire Files Structure 

```
.
├── CMakeLists.txt
├── cmake_modules
│   ├── FindCSparse.cmake
│   └── FindG2O.cmake
├── include
│   └── extra.h
├── README.md
├── ressource
│   └── image
│       ├── 1_depth.png
│       ├── 1.png
│       ├── 2_depth.png
│       └── 2.png
└── src
    ├── extra.cpp
    ├── feature_extraction.cpp
    ├── pose_estimation_2d2d.cpp
    ├── pose_estimation_3d2d.cpp
    ├── pose_estimation_3d3d.cpp
    └── triangulation.cpp



```


<!-- GETTING STARTED -->
## Getting Started

This is a sample code of how you may use triangulation to track motion of object on a map.
To get a local copy up and running follow these simple steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* cmake
  ```sh
  sudo apt-get install cmake
  ```


### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/zoumson/Triangulation.git
   ```
2. Go to the project directory source
   ```sh
   cd Triangulation
   ```
3. Create empty directories `build`, and `bin`
   ```sh
   mkdir build &&  mkdir bin 
   ```
5. Generate the exectutables  and move them  to `bin`
   ```sh
   cd build && cmake .. && make -j4 && cd ..
   ```

<!-- USAGE EXAMPLES -->
### Usage
1. Run for feature extraction 
   ```sh
   ./bin/feature_extraction -i=./ressource/image/1.png -j=./ressource/image/2.png
   ```
2. Run for pose estimation 2D2D 
   ```sh
   ./bin/pose_estimation_2d2d -i=./ressource/image/1.png -j=./ressource/image/2.png
   ```
3. Run for pose estimation 3D2D 
   ```sh
   ./bin/pose_estimation_3d2d -i=./ressource/image/1.png -j=./ressource/image/2.png -d=./ressource/image/1_depth.png
   ```
4. Run for pose estimation 3D3D  
   ```sh
   ./bin/pose_estimation_3d3d -i=./ressource/image/1.png -j=./ressource/image/2.png -d=./ressource/image/1_depth.png -e=./ressource/image/2_depth.png 
   ```
5. Run for triangulation
   ```sh
   ./bin/triangulation -i=./ressource/image/1.png -j=./ressource/image/2.png
   ```

6. Back to the initial file structure configuration
   ```sh
   rm -r bin build 
   ```
<!-- ROADMAP -->
## Roadmap

All the headers files are well docummented, read through the comments

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

Adama Zouma - <!-- [@your_twitter](https://twitter.com/your_username) -->- stargue49@gmail.com

Project Link: [https://github.com/zoumson/Triangulation](https://github.com/zoumson/Triangulation.git)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [Google](https://www.google.com/)
* [Stack Overflow](https://stackoverflow.com/)
* [Github](https://github.com/)




<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: linkedin.com/in/adama-zouma-553bba13a
[product-screenshot]: images/screenshot.png

