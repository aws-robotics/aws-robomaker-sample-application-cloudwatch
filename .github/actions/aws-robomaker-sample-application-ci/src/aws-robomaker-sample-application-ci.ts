import * as path from 'path';
import * as core from '@actions/core';
import * as exec from '@actions/exec';
import { ExecOptions } from '@actions/exec/lib/interfaces';

const fs = require('fs');

const ROS_ENV_VARIABLES: any = {};
const ROS_DISTRO = core.getInput('ros-distro', {required: true});
let PACKAGES = "none"

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

  await exec.exec("bash", [
  "-c",
  `source /opt/ros/${ROS_DISTRO}/setup.bash && printenv`
  ], options)
}

function getExecOptions(listenerBuffers?): ExecOptions {
  var listenerBuffers = listenerBuffers || {};
  const workspaceDir = core.getInput('workspace-dir');
  const execOptions: ExecOptions = {
    cwd: workspaceDir,
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

// If .rosinstall exists, run 'rosws update' and return a list of names of the packages that were added.
async function fetchRosinstallDependencies(): Promise<string[]> {
  let colconListAfter = {stdout: '', stderr: ''};
  let packages: string[] = [];
  // Download dependencies not in apt if .rosinstall exists
  try {
    if (fs.existsSync(path.join(core.getInput('workspace-dir'), '.rosinstall'))) {
      await exec.exec("rosws", ["update"], getExecOptions());
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

    // Update PACKAGES_TO_SKIP_TESTS with the new packages added by 'rosws update'.
    let packages = await fetchRosinstallDependencies();
    PACKAGES = packages.join(" ");
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function build() {
  try {
    await exec.exec("rosdep", ["install", "--from-paths", ".", "--ignore-src", "-r", "-y", "--rosdistro", ROS_DISTRO], getExecOptions());

    console.log(`Build step | packages: ${PACKAGES}`);
    await exec.exec("colcon", ["build", "--build-base", "build", "--install-base", "install"], getExecOptions());
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function bundle() {
  try {
    await exec.exec("colcon", ["bundle", "--build-base", "build", "--install-base", "install", "--bundle-base", "bundle"], getExecOptions());
  } catch (error) {
    core.setFailed(error.message);
  }
}

async function run() {
  await setup();
  await build();
  await bundle();
}

run();
