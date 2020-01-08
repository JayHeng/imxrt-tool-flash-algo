# imxrt-tool-flash-algo
Build tool (IAR, Keil, J-Link) flash algo for i.MXRT | 收集i.MXRT主流开发工具的flash算法源工程

> IAR EWARM v8.40.2
> Keil MDK v5.27
> J-Link v6.52e
> MCUXpresso IDE v11.0.0_2495_alpha

<table><tbody>
    <tr>
        <th>RT</th>
        <th>IDE</th>
        <th>Board</th>
        <th>Flash</th>
        <th>Status</th>
    </tr>
    <tr>
        <td rowspan="5">i.MXRT600 A0</td>
        <td>IAR EWARM</td>
        <td>/</td>
        <td>/</td>
        <td>/</td>
    </tr>
    <tr>
        <td rowspan="3">Keil MDK/J-Link</td>
        <td>NXP RT600_BGA176_Validation_Board_Rev.0</td>
        <td>U8 - W25Q64FW</td>
        <td>Done, FlashPrg.c</td>
    </tr>
    <tr>
        <td>NXP MIMXRT685-EVK_Rev.B</td>
        <td>U19 - MX25UM51345GXDI00</td>
        <td>/</td>
    </tr>
    <tr>
        <td>MSFT xProject V1</td>
        <td>W25Q16FWUXIE (no SFDP)</td>
        <td>Done, FlashPrg_w25q_nosfdp_v2.c</td>
    </tr>
    <tr>
        <td>MCUXpresso IDE</td>
        <td>/</td>
        <td>/</td>
        <td>/</td>
    </tr>
</table>
