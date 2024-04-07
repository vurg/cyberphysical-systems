# Group 21
**DIT639 Project: Cyber Physical Systems and Systems of Systems**

### Current Build Status
[![pipeline status](https://git.chalmers.se/courses/dit638/students/2024-group-21/badges/main/pipeline.svg)](https://git.chalmers.se/courses/dit638/students/2024-group-21/-/pipelines)

## Installation
To install our system, complete the following steps:

1. Clone this repository into your preferred directory with ```git clone git@git.chalmers.se:courses/dit638/students/2024-group-21.git```
2. Run ```cd a5``` and open helloworld.cpp. Modify "LastName, FirstName" to your name.
3. docker build -t CID/example:latest -f Dockerfile .
4. docker run --rm CID/example:latest 42

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

## Contributers
<table>
  <tr>
    <td align="center"><img src="https://secure.gravatar.com/avatar/3056b6827d3d959ea87306c4d2dd0c6a?s=800&d=identicon" width="100px;"/><br/><sub><b>Daniel Van Den Heuvel</b></sub><br>@heuvel</td>
    <td align="center"><img src="https://secure.gravatar.com/avatar/3271ba4e481b7c393b650b96a17344d0?s=800&d=identicon" width="100px;"/><br/><sub><b>Kai Rowley</b></sub><br>@rowley</td>
    <td align="center"><img src="https://secure.gravatar.com/avatar/82899676cb5f15c859ed9bd18921b3033716285c1331ed8406c725e91f95bd80?s=800&d=identicon" width="100px;"/><br/><sub><b>Sam Hardingham</b></sub><br>@samha</td>
    <td align="center"><img src="https://git.chalmers.se/uploads/-/system/user/avatar/3455/avatar.png?width=400" width="100px;"/><br/><sub><b>Nasit Vurgun</b></sub><br>@nasit</td>
  </tr>
 </table> 
