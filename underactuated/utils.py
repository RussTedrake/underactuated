import os

from pydrake.all import namedview

running_as_test = False


def set_running_as_test(value):
    global running_as_test
    running_as_test = value


def FindResource(filename):
    return os.path.join(os.path.dirname(__file__), filename)


def ConfigureParser(parser):
    """Add the underactuated/package.xml index to the given pydrake Parser."""
    package_xml = os.path.join(os.path.dirname(__file__), "package.xml")
    parser.package_map().AddPackageXml(filename=package_xml)


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


def MakeNamedViewPositions(
    mbp, view_name, add_suffix_if_single_position=False
):
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


def MakeNamedViewVelocities(
    mbp, view_name, add_suffix_if_single_velocity=False
):
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
