import logging
import click
import yaml

logging.basicConfig(level=logging.INFO)

from kittiground import DEFAULT_CONFIG_FILE
from kittiground.kittiground import KittiGround


@click.group()
def cli():
    """Polyliadr with KITTI Dataset"""

@cli.command()
@click.argument('input', type=click.File('r'), default=str(DEFAULT_CONFIG_FILE))
def run(input):
    """Run ground detector on KITTI Dataset.

    input is a file path to a yaml configuration file, Default=config/default.yaml
    """
    click.echo('Hello World!')
    try:
        config = yaml.safe_load(input)
    except yaml.YAMLError as exc:
        logging.exception("Error parsing yaml")
    kg = KittiGround(config)
    kg.run()

