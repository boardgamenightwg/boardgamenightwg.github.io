+++
title = "Conda Packaging - Polyglot, Project Scoped, Platform Agnostic"
date = 2023-09-27
type = "post"
in_search_index = true
draft = true
[taxonomies]
tags = ["Packaging"]
+++

## TODO

- [ ] Use `rattler-build` with hard-coded path
- [ ] Upload packages to prefix.dev
- [ ] Consume packages from prefix.dev
- [ ] Generate recipe from package.xml using `vinca`
- [ ] Use `boa` with a channel (condarc file)
- [ ] Setup and test with local `quetz` server

## Experimenting with cpp_polyfills

I'm using the repo cpp_polyfills to test packaging with conda. Here is my WIP
https://github.com/PickNikRobotics/cpp_polyfills/pull/2

## Pixi

[Read the docs](https://prefix.dev/docs/pixi/overview)

* Package management tool that aims to give a `cargo` like experience.
* Written in Rust, takes a `pixi.toml` config file
* Install dependencies and generate lock files
* Create shell enviroment with dependencies
* Create `tasks`, aliases for scripts run in the shell enviroment
* Upload packages to prefix.dev or another channel
* Building packages is an [open pr](https://github.com/prefix-dev/pixi/pull/85)

## Prefix.dev

[Read the docs](https://prefix.dev/docs/prefix/overview)

Website that offers hosting of private and public channels of packages.
Currently public and private channels are free, in the future they will be paid.

## rattler-build

[Read the docs](https://prefix-dev.github.io/rattler-build/highlevel.html)

A tool for building conda packages.
A replacement for `conda-build` and `boa`.
Takes in `recipe.yaml` file(s) and produces conda package(s).

Takes `--channel <channel_name>` arguments which allows you to consume dependencies from multiple channel sources.

## boa

[Read the docs](https://boa-build.readthedocs.io/en/latest/)

Boa is a package build tool for `.conda` packages.
I have not figured out how to use boa with a channel.

## Quetz

[Read the docs](https://quetz.readthedocs.io/en/latest/)

Open source server for conda packages.

## Links

- [Pixi Crispy Doom](https://github.com/baszalmstra/pixi-crispy-doom) - What it sounds like!


# Notes

Deployment:

https://github.com/conda/constructor
Creates installers from conda packages

Rattler crates are underlying code for pixi
