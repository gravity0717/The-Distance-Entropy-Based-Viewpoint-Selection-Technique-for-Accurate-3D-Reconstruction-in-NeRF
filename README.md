# The Distance-Entropy Based Viewpoint Selection Technique for Accurate 3D Reconstruction in NeRF

## Introduction
3D reconstruction requires images from various viewpoints. For large scenes or complex objects, more viewpoints are necessary, and acquiring these images requires skill. When inexperienced beginners capture the images, they may encounter difficulties in reconstruction, potentially leading to failure and requiring significant time and effort. Existing guidelines are heuristic and lack specific methods. To address this issue, the project focuses on view-selection and uncertainty as key concepts.
You can access published paper [here](https://jkros.org/_common/do.php?a=full&b=33&bidx=3565&aidx=39610).

3D Reconstruction에는 다양한 시점에서의 이미지를 필요로 한다. 거대한 장면이나, 복잡한 물체에 대해서는 더 많은 시점에서의 이미지를 필요로하며, 이미지를 획득하는데에 숙련도가 요구된다. 숙련되지 않은 초보자가 이미지를 획득했을때, 복원하는데 어려움을 겪고 실패할 수 있으며 많은 시간과 노력을 필요로 한다. 기존에 주어진 가이드는 휴리스틱하고 구체적인 방법이 제시되지 않았다. 이러한 문제를 해결하고자 *View-selection*, *Uncertainty*를 keyword로 잡고 프로젝트를 진행한다.

## Related Work
Next-Best-View

## Contributor 
Jinwon Choi (randomwons)
Jun Hyeok Choi (cjh1995-ros) 
Chanho Seo (gravity0717)

## TODO
- [x] Instant-NGP 코드 분석
- [x] Entropy 랜더링
- [x] Camera 연결
- [x] Camera pose estimation (ArUco)
- [x] Candidate view 생성
- [x] View-selection
- [x] View pose rendering
- [x] Evaluation

## Demo 
![demo](asset/demo.gif)

## License and Citation

```bibtex
@article{mueller2022instant,
    author = {Thomas M\"uller and Alex Evans and Christoph Schied and Alexander Keller},
    title = {Instant Neural Graphics Primitives with a Multiresolution Hash Encoding},
    journal = {ACM Trans. Graph.},
    issue_date = {July 2022},
    volume = {41},
    number = {4},
    month = jul,
    year = {2022},
    pages = {102:1--102:15},
    articleno = {102},
    numpages = {15},
    url = {https://doi.org/10.1145/3528223.3530127},
    doi = {10.1145/3528223.3530127},
    publisher = {ACM},
    address = {New York, NY, USA},
}
```

Copyright © 2022, NVIDIA Corporation. All rights reserved.

This work is made available under the Nvidia Source Code License-NC. Click [here](LICENSE.txt) to view a copy of this license.
