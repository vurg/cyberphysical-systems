# Group 21
**DIT639 Project: Cyber Physical Systems and Systems of Systems**

### Current Build Status
[![pipeline status](https://git.chalmers.se/courses/dit638/students/2024-group-21/badges/main/pipeline.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-21/-/pipelines)
[![coverage report](https://git.chalmers.se/courses/dit638/students/2024-group-21/badges/main/coverage.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-21/-/commits/main)
[![Latest Release](https://git.chalmers.se/courses/dit638/students/2024-group-21/-/badges/release.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-21/-/releases)


## Acknowledgments
Special thanks to @christian.berger for his contributions to the foundational libraries used in our project, including OpenDLV, libcluon, and the opencvv template file, which have been instrumental in the development of our cyber-physical systems project.

## Installation
To install our system, complete the following steps:

1. Clone this repository into your preferred directory with ```git clone git@git.chalmers.se:courses/dit638/students/2024-group-21.git```
2. Build the project (steering wheel angle calculator microservice) using Dockerfile:
```
cd cpp-opencv
docker build -f Dockerfile -t anglecalculator .
```
3. Run microservices in same directory as recording files:
```
// terminal 1: openDLV - new terminal window (ctrl+alt+t):
docker run --rm -i --init --net=host --name=opendlv-vehicle-view -v $PWD:/opt/vehicle-view/recordings -v /var/run/docker.sock:/var/run/docker.sock -p 8081:8081 chrberger/opendlv-vehicle-view:v0.0.64

// terminal 2: h264decoder - new terminal tab (ctrl+shift+t):
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264decoder:v0.0.5 --cid=253 --name=img

// terminal 3: template project microservice (steering wheel angle calculator) - new terminal tab (ctrl+shift+t):
xhost +
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp anglecalculator:latest --cid=253 --name=img --width=640 --height=480 --verbose 
```

## Adding New Features
We will be following the basic git workflow and ensure usual code reviews.

**Merge Conflict Rule:**
Pull main and merge main to the branch you are on. Solve merge conflicts locally!

**Branch Creation:**
If issue is "#1 Parse recording file", you would create the branch after:
1-parse-recording-file

### Issue Body:
**User story:**
As a ___persona___, I want to ___action___ so that ___result___.

**Acceptance criteria:**
- [ ] do X
- [ ] do Y
- [ ] do Z

### Commit Format:
```
git commit -m "#1 <<Present tense verb noun - what it does>>" -m "<<longer commit message e.g. Co-authored by rowley@student.chalmers.se optional... >>"
```
Concrete Example:
```
git commit -m "#1 Implements parsing for rec files" -m "Works using xyz encoding. Co-authored by rowley@student.chalmers.se"
```

## Fixing Bugs
We plan to fix unexpected behaviour by creating a respective issue for that specific bug and following the previously mentioned git workflow in creating a new branch to address that problem.


### Code-Fix Issue Format:
This format is applied when issuing a code change issue for e.g refactoring, renaming or typo-fixing intentions.

**Title**: Header for issue of concern highlighting the intended fix.

**Body**: Description of what the issue is and its fix is. ***Optional*** Address issue/s of which are concerned.

***Optional*** **Tags**: Relevant tags, either one or many in combination pertaining to issue at hands e.g. Fix, Fix + Typo, Refactor etc.

Concrete Example:
```
<title> Improve error handling in file processing

<body> The file processing module does not handle errors 
properly when encountering invalid input files. Improve error 
handling by adding more informative error messages and logging 
errors to the standard output.

<tags> Fix, Error Output 
```
## Merge Request Format

This format is applied when requesting a branch to be merged with another branch via a Merge Request.

**Title**: Latest commit message header

**Body**: Closes #>>Issue Number<

Concrete Example:
```
<title> #50 Manage datatypes for redueced memory uptake

<body> Closes #50

```

## Contributers
<table>
  <tr>
    <td align="center"><img src="https://secure.gravatar.com/avatar/3056b6827d3d959ea87306c4d2dd0c6a?s=800&d=identicon" width="100px;"/><br/><sub><b>Daniel Van Den Heuvel</b></sub><br>@heuvel</td>
    <td align="center"><img src="https://secure.gravatar.com/avatar/3271ba4e481b7c393b650b96a17344d0?s=800&d=identicon" width="100px;"/><br/><sub><b>Kai Rowley</b></sub><br>@rowley</td>
    <td align="center"><img src="https://secure.gravatar.com/avatar/82899676cb5f15c859ed9bd18921b3033716285c1331ed8406c725e91f95bd80?s=800&d=identicon" width="100px;"/><br/><sub><b>Sam Hardingham</b></sub><br>@samha</td>
    <td align="center"><img src="https://git.chalmers.se/uploads/-/system/user/avatar/3455/avatar.png?width=400" width="100px;"/><br/><sub><b>Nasit Vurgun</b></sub><br>@nasit</td>
  </tr>
 </table> 
