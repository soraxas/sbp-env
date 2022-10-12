#!/usr/bin/env python

import sbp_env

if __name__ == "__main__":
    # The entry point of the planning scene module from the cli
    default_arguments = sbp_env.generate_args_main()

    environment = sbp_env.env.Env(args=default_arguments)
    environment.run()

__doc__ = sbp_env.__doc__
