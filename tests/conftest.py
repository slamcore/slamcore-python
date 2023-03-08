# ******************************************************************************
#
#  Slamcore Confidential
#  ---------------------
#
#  Slamcore Limited
#  All Rights Reserved.
#  (C) Copyright 2023
#
#  NOTICE:
#
#  All information contained herein is, and remains the property of Slamcore
#  Limited and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Slamcore Limited and its suppliers and
#  may be covered by patents in process, and are protected by trade secret or
#  copyright law. Dissemination of this information or reproduction of this
#  material is strictly forbidden unless prior written permission is obtained
#  from Slamcore Limited.
#
# ******************************************************************************

import pytest
import os


def pytest_addoption(parser):
    parser.addoption("--dataset", action="store", type=str)
    parser.addoption("--config", action="store", type=str)


def pytest_generate_tests(metafunc):
    dataset = os.path.expanduser(metafunc.config.option.dataset)
    config = metafunc.config.option.config
    if config is not None:
        config = os.path.expanduser(config)
    metafunc.parametrize("dataset, config", [(dataset, config)])
