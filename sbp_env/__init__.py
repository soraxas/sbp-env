from . import env, collisionChecker, engine, randomness, visualiser, planners, samplers
from .__entry_point import generate_args, generate_args_main


def run_cli():
    # The entry point of the planning scene module from the cli
    default_arguments = generate_args_main()

    environment = env.Env(args=default_arguments)
    environment.run()
