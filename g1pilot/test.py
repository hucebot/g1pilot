from __future__ import annotations

import logging
import time
from typing import Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

def _rpc_get_int(client: LocoClient, api_id: int) -> Optional[int]:
    try:
        code, data = client._Call(api_id, "{}")  # type: ignore[attr-defined]
        if code == 0 and data:
            import json
            return json.loads(data).get("data")
    except Exception:
        pass
    return None

def _fsm_id(client: LocoClient) -> Optional[int]:
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_ID)

def _fsm_mode(client: LocoClient) -> Optional[int]:
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_MODE)

def hanger_boot_sequence(
    iface: str = "eth1",
    step: float = 0.02,
    max_height: float = 0.5,
    logger: Optional[logging.Logger] = None,
) -> LocoClient:
    if logger is None:
        logging.basicConfig(level=logging.INFO, format="%(message)s")
        logger = logging.getLogger("hanger_boot")

    ChannelFactoryInitialize(0, iface)
    bot = LocoClient()
    bot.SetTimeout(10.0)
    bot.Init()

    try:
        cur_id = _fsm_id(bot)
        cur_mode = _fsm_mode(bot)
        if cur_id == 200 and cur_mode is not None and cur_mode != 2:
            logger.info(
                "Robot already in balanced stand (FSM 200, mode %s) – skipping boot sequence.",
                cur_mode,
            )
            return bot
    except Exception:
        pass

    def show(tag: str) -> None:
        logger.info("%-12s → FSM %s   mode %s", tag, _fsm_id(bot), _fsm_mode(bot))

    bot.Damp(); show("damp")
    bot.SetFsmId(4); show("stand_up")

    while True:
        height = 0.0
        while height < max_height:
            height += step
            bot.SetStandHeight(height)
            show(f"height {height:.2f} m")
            if _fsm_mode(bot) == 0 and height > 0.2:
                break
        if _fsm_mode(bot) == 0:
            break
        logger.warning(
            "Feet still unloaded (mode %s) after reaching %.2f m.\n"
            "Adjust hanger height (raise/lower until the soles are just in\n"
            "contact with the ground) then press <Enter> to try again…",
            _fsm_mode(bot),
            height,
        )
        try:
            bot.SetStandHeight(0.0)
            show("reset")
        except Exception:
            pass
        input()

    bot.BalanceStand(0); show("balance")
    bot.SetStandHeight(height); show("height✔")
    bot.Start(); show("start")
    return bot

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    bot = hanger_boot_sequence(iface="eth1")
    print(f"Robot is now in FSM {bot.GetFsmId()} (mode {_fsm_mode(bot)})")
