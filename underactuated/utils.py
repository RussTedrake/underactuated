import os
from datetime import date
from urllib.request import urlretrieve
from warnings import warn

from pydrake.all import PackageMap, namedview
from pydrake.common import GetDrakePath

running_as_test = False


def _set_running_as_test(value):
    """[INTERNAL USE ONLY]: Set the global variable `running_as_test` to
    `value`.

    This method is used by the build system; it is not intended for general
    use.
    """
    global running_as_test
    running_as_test = value


def FindResource(filename):
    """Returns the absolute path to the given filename relative to the
    underactuated module."""
    return os.path.join(os.path.dirname(__file__), filename)


def FindDataResource(filename: str):
    """
    Returns the absolute path to the given filename relative to the data directory; fetching it from a remote host if necessary.
    """
    data = os.path.join(os.path.dirname(os.path.dirname(__file__)), "book", "data")
    if not os.path.exists(data):
        os.makedirs(data)
    path = os.path.join(data, filename)
    if not os.path.exists(path):
        print(f"{path} was not found locally; downloading it now...")
        urlretrieve(f"https://underactuated.csail.mit.edu/data/{filename}", path)
    return path


def ConfigureParser(parser):
    """Add the underactuated module packages to the given Parser."""
    package_xml = os.path.join(os.path.dirname(__file__), "package.xml")
    parser.package_map().AddPackageXml(filename=package_xml)
    # Add spot_description
    parser.package_map().AddRemote(
        package_name="spot_description",
        params=PackageMap.RemoteParams(
            urls=[
                f"https://github.com/bdaiinstitute/spot_ros2/archive/097f46bd7c5b1d6bf0189a895c28ae0a90d287b1.tar.gz"
            ],
            sha256=("59df2f0644bd7d937365e580a6d3b69822e7fb0358dc58456cd0408830641d88"),
            strip_prefix="spot_ros2-097f46bd7c5b1d6bf0189a895c28ae0a90d287b1/spot_description/",
        ),
    )


def Rgba2Hex(rgb):
    """
    Turn a list of R,G,B elements (any indexable list of >= 3 elements will
    work), where each element is specified on range [0., 1.], into the
    equivalent 24-bit value 0xRRGGBB.
    """
    val = 0
    for i in range(3):
        val += (256 ** (2 - i)) * int(255 * rgb[i])
    return val


def MakeNamedViewPositions(mbp, view_name, add_suffix_if_single_position=False):
    print(
        "MakeNamedViewPositions() is deprecated.  Use `namedview(view_name, plant.GetPositionNames())` instead)"
    )
    return namedview(
        view_name,
        plant.GetPositionNames(
            add_model_instance_prefix=False,
            always_add_suffix=not add_suffix_if_single_position,
        ),
    )


def MakeNamedViewVelocities(mbp, view_name, add_suffix_if_single_velocity=False):
    print(
        "MakeNamedViewVelocities() is deprecated.  Use `namedview(view_name, plant.GetVelocityNames())` instead)"
    )
    return namedview(
        view_name,
        plant.GetVelocityNames(
            add_model_instance_prefix=False,
            always_add_suffix=not add_suffix_if_single_velocity,
        ),
    )


def MakeNamedViewState(mbp, view_name):
    print(
        "MakeNamedViewState() is deprecated.  Use `namedview(view_name, plant.GetStateNames())` instead)"
    )
    return namedview(
        view_name,
        plant.GetStateNames(add_model_instance_prefix=False),
    )


def MakeNamedViewActuation(mbp, view_name):
    print(
        "MakeNamedViewActuation() is deprecated.  Use `namedview(view_name, plant.GetActuatorNames())` instead)"
    )
    return namedview(
        view_name,
        plant.GetActuatorNames(add_model_instance_prefix=False),
    )


def GetDrakeVersion() -> str:
    """Returns a string representing the Drake version, if available, or "source"."""
    import importlib.metadata

    try:
        return importlib.metadata.version("drake")
    except importlib.metadata.PackageNotFoundError:
        return "source"


def DrakeVersionGreaterThan(minimum_date: date):
    """Check that the Drake version is at least `minimum_date`."""
    version_dates = {
        "1.13.0": date(year=2023, month=2, day=14),
        "1.14.0": date(year=2023, month=3, day=15),
        "1.15.0": date(year=2023, month=4, day=18),
        "1.16.0": date(year=2023, month=5, day=18),
        "1.17.0": date(year=2023, month=5, day=23),
        "1.18.0": date(year=2023, month=6, day=20),
        "1.19.0": date(year=2023, month=7, day=13),
        "1.20.0": date(year=2023, month=8, day=16),
        "1.21.0": date(year=2023, month=9, day=14),
        "1.22.0": date(year=2023, month=10, day=16),
        "1.23.0": date(year=2023, month=11, day=17),
        "1.24.0": date(year=2023, month=12, day=18),
    }
    drake_version = GetDrakeVersion()
    # A source install won't have package metadata; assume it's new enough.
    if drake_version == "source":
        return
    # Nightly builds look like "0.0.YYYYMMDD" via importlib.metadata.
    if drake_version.startswith("0.0."):
        date_str = drake_version[4:]
        if len(date_str) == 8:
            drake_date = date(
                year=int(date_str[:4]),
                month=int(date_str[4:6]),
                day=int(date_str[6:8]),
            )
        else:
            warn(f"Unrecognized drake version {drake_version}")
            return
    elif drake_version in version_dates:
        drake_date = version_dates[drake_version]
    else:
        warn(f"Unrecognized drake version {drake_version}")
        return
    if drake_date < minimum_date:
        raise RuntimeError(
            f"You need to update your Drake version. Python is using the Drake "
            f"installation in {GetDrakePath()}. This installation was from a "
            f"nightly build on {drake_date}, but this method requires Drake "
            f"from at least {minimum_date}."
        )
