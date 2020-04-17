import * as path from 'path';
import * as core from '@actions/core';
import * as exec from '@actions/exec';
import { ExecOptions } from '@actions/exec/lib/interfaces';

const fs = require('fs');

const ROS_DISTRO = core.getInput('ros-distro', {required: true});
let GAZEBO_VERSION = core.getInput('gazebo-version');
let SAMPLE_APP_VERSION = '';
const WORKSPACE_DIRECTORY = core.getInput('workspace-dir');
let PACKAGES = "none"
const ROS_ENV_VARIABLES: any = {};

async function loadROSEnvVariables() {
  const options = {
    listeners: {
      stdout: (data: Buffer) => {
        const lines = data.toString().split("\n");
        lines.forEach(line => {
          if (line.trim().length === 0) return;
          const contents = line.trim().split("=");
          ROS_ENV_VARIABLES[contents[0]] = contents.slice(1).join("=");
        });
      }
    }
  };

  await exec.exec("bash", ["-c", `source /opt/ros/${ROS_DISTRO}/setup.bash && printenv`], options)
}

function getExecOptions(listenerBuffers?): ExecOptions {
  var listenerBuffers = listenerBuffers || {};
  const execOptions: ExecOptions = {
    cwd: WORKSPACE_DIRECTORY,
    env: Object.assign({}, process.env, ROS_ENV_VARIABLES)
  };
  if (listenerBuffers) {
    execOptions.listeners = {
      stdout: (data: Buffer) => {
        listenerBuffers.stdout += data.toString();
      },
      stderr: (data: Buffer) => {
        listenerBuffers.stderr += data.toString();
      }
    };
  }
  return execOptions
}

async function getSampleAppVersion() : Promise<string> {
  let grepAfter = {stdout: '', stderr: ''};
  let version = '';
  try {
    await exec.exec("bash", [
        "-c",
         "find ../robot_ws -name package.xml -exec grep -Po '(?<=<version>)[^\\s<>]*(?=</version>)' {} +"],
      getExecOptions(grepAfter));
    version = grepAfter.stdout.trim();
  } catch(err) {
    console.error(err);
  }
  return Promise.resolve(version);
}

// If .rosinstall exists, run 'rosws update' and return a list of names of the packages that were added.
async function fetchRosinstallDependencies(): Promise<string[]> {
  let colconListAfter = {stdout: '', stderr: ''};
  let packages: string[] = [];
  // Download dependencies not in apt if .rosinstall exists
  try {
    for (let workspace of ["robot_ws", "simulation_ws"]) {
      if (fs.existsSync(path.join(workspace, '.rosinstall'))) {
        await exec.exec("rosws", ["update", "-t", workspace]);
      }
    }
    if (fs.existsSync(path.join(WORKSPACE_DIRECTORY, '.rosinstall'))) {
      await exec.exec("colcon", ["list", "--names-only"], getExecOptions(colconListAfter));
      const packagesAfter = colconListAfter.stdout.split("\n");
      packagesAfter.forEach(packageName => {
        packages.push(packageName.trim());
      });
    }
  } catch(err) {
    console.error(err);
  }
  return Promise.resolve(packages);
}

async function setup() {
  try {
    await exec.exec("apt-key", ["adv", "--fetch-keys", "http://packages.osrfoundation.org/gazebo.key"]);

    const aptPackages = [
      "zip",
      "cmake",
      "lcov",
      "libgtest-dev",
      "python-pip",
      "python-rosinstall",
      "python3-colcon-common-extensions",
      "python3-pip",
      "python3-apt"
    ];

    const python3Packages = [
      "setuptools",
      "colcon-bundle",
      "colcon-ros-bundle"
    ];

    await exec.exec("sudo", ["apt-get", "update"]);
    await exec.exec("sudo", ["apt-get", "install", "-y"].concat(aptPackages));
    await exec.exec("sudo", ["pip3", "install", "-U"].concat(python3Packages));

    await exec.exec("rosdep", ["update"]);

    await loadROSEnvVariables();

    SAMPLE_APP_VERSION = await getSampleAppVersion();
    console.log(`Sample App version found to be: ${SAMPLE_APP_VERSION}`);

    // Update PACKAGES_TO_SKIP_TESTS with the new packages added by 'rosws update'.
    let packages = await fetchRosinstallDependencies();
    PACKAGES = packages.join(" ");
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function setup_gazebo9() {
  try {
    const gazebo9_apt_file = "/etc/apt/sources.list.d/gazebo-stable.list";
    await exec.exec("sudo", ["rm", "-f", gazebo9_apt_file]);
    await exec.exec("bash", ["-c", `echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb_release -cs\` main" >> ${gazebo9_apt_file}`]);
    await exec.exec("sudo", ["apt-get", "update"]);

    if (ROS_DISTRO == "kinetic") {
      const gazebo9_rosdep_file = "/etc/ros/rosdep/sources.list.d/00-gazebo9.list";
      await exec.exec("sudo", ["rm", "-f", gazebo9_rosdep_file]);
      await exec.exec("bash", ["-c", `echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/gazebo9/gazebo.yaml" >> ${gazebo9_rosdep_file}`]);
      await exec.exec("bash", ["-c", `echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/gazebo9/releases/indigo.yaml indigo" >> ${gazebo9_rosdep_file}`]);
      await exec.exec("bash", ["-c", `echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/gazebo9/releases/jade.yaml jade" >> ${gazebo9_rosdep_file}`]);
      await exec.exec("bash", ["-c", `echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/gazebo9/releases/kinetic.yaml kinetic" >> ${gazebo9_rosdep_file}`]);
      await exec.exec("bash", ["-c", `echo "yaml https://github.com/osrf/osrf-rosdep/raw/master/gazebo9/releases/lunar.yaml lunar" >> ${gazebo9_rosdep_file}`]);
      await exec.exec("rosdep", ["update"]);
    }
   } catch (error) {
    core.setFailed(error.message);
  }
}

async function prepare_sources() {
  try {
    const sourceIncludes = [
      "robot_ws",
      "simulation_ws",
      "LICENSE*",
      "NOTICE*",
      "README*",
      "roboMakerSettings.json"
    ];

    const sourceIncludesStr = sourceIncludes.join(" ");
    await exec.exec("bash", ["-c", `zip -r sources.zip ${sourceIncludesStr}`]);
    await exec.exec("bash", ["-c", `tar cvzf sources.tar.gz ${sourceIncludesStr}`]);
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function build() {
  try {
    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y", "--rosdistro", ROS_DISTRO], getExecOptions());

    console.log(`Building the following packages: ${PACKAGES}`);
    await exec.exec("colcon", ["build", "--build-base", "build", "--install-base", "install"], getExecOptions());
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function bundle() {
  try {
    await exec.exec("colcon", ["bundle", "--build-base", "build", "--install-base", "install", "--bundle-base", "bundle"], getExecOptions());
    await exec.exec("mv", ["bundle/output.tar", `../${WORKSPACE_DIRECTORY}.tar`], getExecOptions());
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function run() {
  console.log(`ROS_DISTRO: ${ROS_DISTRO}`);
  console.log(`GAZEBO_VERSION: ${GAZEBO_VERSION}`);
  console.log(`WORKSPACE_DIRECTORY: ${WORKSPACE_DIRECTORY}`);

  await setup();
  if (ROS_DISTRO == "kinetic" && (GAZEBO_VERSION == "" || GAZEBO_VERSION == "7")) {
    GAZEBO_VERSION = "7";
  } else if (ROS_DISTRO == "kinetic" && GAZEBO_VERSION == "9") {
    await setup_gazebo9();
  } else if (ROS_DISTRO == "melodic" && (GAZEBO_VERSION == "" || GAZEBO_VERSION == "9")) {
    GAZEBO_VERSION = "9";
    await setup_gazebo9();
  } else if (ROS_DISTRO == "dashing" && (GAZEBO_VERSION == "" || GAZEBO_VERSION == "9")) {
    GAZEBO_VERSION = "9";
    await setup_gazebo9();
  } else {
    core.setFailed(`Invalid ROS and Gazebo combination`);
  }
  await prepare_sources();
  await build();
  await bundle();

  core.setOutput('ros-distro', ROS_DISTRO)
  core.setOutput('gazebo-version', "gazebo"+GAZEBO_VERSION)
  core.setOutput('sample-app-version', SAMPLE_APP_VERSION);
}

run();
