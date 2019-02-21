# ego-app-builder #

The Ego App Builder repository is a fork of the Godot source from the open source project maintained in GitHub here: https://github.com/godotengine/godot.  The source is left largely 
unchanged here, but it includes below the modules directory a git sub-module for each of the Godot Native Modules that are included in the Ego Apps such as the gRPC Module and the 
Extensions Module.

### Overview ###

The build scripts are enhanced to publish artifacts to the cloud-hosted Artifactory account with a specific version.

For more details refer to the [Confluence page](https://confluence.corp.imvu.com/display/EGO/Ego+Engine+and+Application+Continuous+Integration#EgoEngineandApplicationContinuousIntegration-EgoAppBuilder).

### Setup ###

If you do not have write access to this repository and need to do development work on the Ego App, then submit an IT Request asking to be added to 
the **Withme: Engine Project: Write group**.

You will use the latest version of the **Godot Editor** produced by the ego-builder and published to Artifactory.  You can get appropriate Godot editor executable 
for your development platform by logging into Artifactory using your IMVU credentials here: https://withme.jfrog.io/withme/webapp/

TODO: Document the following topics

* Summary of set up
* Configuration
* Dependencies
* How to run tests
* Deployment instructions

### Contribution guidelines ###

TODO: Document the following topics

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin: Erik K. Worth <eworth@imvu.com>
* Other community or team contact: Hein-Pieter van Braam-Stewart <hp@prehensile-tales.com>