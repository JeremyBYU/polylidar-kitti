from kittiground.kittiground import KittiGround
from kittiground import DEFAULT_CONFIG_FILE
import logging
from pathlib import Path

import click
import yaml
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.INFO)


sns.set()


@click.group()
def cli():
    """Polylidar3D with KITTI Dataset"""


def create_time_df(df, drive):
    df['frame_idx'] = df.index
    df['drive'] = drive
    df = df[['drive', 'frame_idx', 'n_points', 't_outlier', 't_rotation',
             't_polylidar_all', 't_polylidar_mesh', 't_polylidar_bilateral', 't_polylidar_planepoly', 't_polyfilter']]
    return df


@cli.command()
@click.argument('input', type=click.File('r'))
def plot(input):
    df = pd.read_csv(input)
    df = df[['drive', 'frame_idx', 't_outlier', 't_polylidar_all', 't_polyfilter']]
    df_melt = pd.melt(df, id_vars=['drive', 'frame_idx'], var_name=['Group'], value_vars=[
                      't_outlier', 't_polylidar_all', 't_polyfilter'], value_name='Execution Time (ms)')
    facet_kwargs = dict(gridspec_kws=dict(hspace=.05))
    g = sns.catplot(x="Group", y="Execution Time (ms)", col='drive', data=df_melt, col_wrap=4, kind='boxen')
    g.set(ylim=(0, 20))
    # ax = sns.boxplot(x="Group", y="Execution Time", data=df_melt)
    plt.subplots_adjust(hspace=0.2, wspace=0.05)
    plt.show()


@cli.command()
@click.argument('input', type=click.File('r'))
def mean(input):
    df = pd.read_csv(input)
    df = df[['drive', 'frame_idx', 'n_points','t_outlier', 't_polylidar_mesh', 't_polylidar_bilateral', 't_polylidar_planepoly', 't_polyfilter']]
    print(df.mean())
    print(df.count())


@cli.command()
@click.argument('input', type=click.File('r'), default=str(DEFAULT_CONFIG_FILE))
def run(input):
    """Run ground detector on KITTI Dataset.

    input is a file path to a yaml configuration file, Default=config/default.yaml
    """
    try:
        config = yaml.safe_load(input)
    except yaml.YAMLError as exc:
        logging.exception("Error parsing yaml")
    if isinstance(config['drive'], list):
        all_drives = config['drive'].copy()
    else:
        all_drives = [config['drive']]

    all_dfs = []
    for drive in all_drives:
        config['drive'] = drive
        kg = KittiGround(config)
        df = kg.run()
        logging.info("Finished Drive: %r", drive)
        df = create_time_df(df, drive)
        all_dfs.append(df)

    merged_df = pd.concat(all_dfs)
    if config['timings']['active']:
        fpath = Path(config['timings']['directory'])
        fpath = fpath / "combined.csv"
        merged_df.to_csv(str(fpath), index=False)
