"""Reproduce the small measured-input fixture; no device or network access."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def save(name, value):
    (HERE/'inputs'/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def main():
    old = json.loads((HERE.parent/'piper_clickbell_workcell_20261025/inputs/registration.json').read_text(encoding='utf-8'))
    capture = json.loads((HERE/'inputs/capture_summary.json').read_text(encoding='utf-8'))
    save('intrinsics.json', dict(K=capture['K'], D=capture['D'], image_size=[1280,720],
                                 source='Frozen resume10 RGB gate 20261025_213538; no depth'))
    save('targets.json', dict(targets=dict(dictionary='DICT_5X5_100', boards=dict(
        A=dict(cols=7, rows=5, square_mm=36, marker_mm=27, first_id=0, legacy_pattern=False)))))
    top = [p[:2]+[0.] for p in old['scene']['table_top_board_m']]
    # Frozen manual outline on the first image. This is the visible bell outline,
    # not a box-corner annotation; proxy/real shape mismatch remains in the score.
    contour = [[550,302],[567,305],[580,315],[588,328],[597,343],[601,359],
               [594,375],[578,386],[554,393],[532,390],[515,381],[504,365],
               [503,351],[510,333],[520,315],[535,305]]
    save('measurements.json', dict(schema='rp-workcell-measurements/1',
         id='piper_rgb_workcell_20261026', units='m', targets='targets.json',
         intrinsics='intrinsics.json', level_table=True,
         scale_evidence='沿用现行工作台实测：板 A 7×5、格长36mm；桌面1800×805×30mm；铃85×85×45mm、板系(37,-142)mm。未重新测量；将历史桌面z=8mm修正为纸贴桌面z=0，显式记录。',
         images=[dict(path='head_0.png', object_contours_px=dict(bell=contour),
                      diagnostic_bell_ground_center_px=[551,366]), dict(path='head_1.png')],
         table=dict(top_corners_board_m=top, thickness_m=.03),
         objects=[dict(name='bell', category='bell', dimensions_m=[.085,.085,.045],
                       base_center_board_m=[.037,-.142,0], yaw_deg=0, graspable=False)],
         measurement_provenance=dict(source='piper_clickbell_workcell_20261025/inputs/registration.json',
             source_commit='8b249d4', historical_table_z_m=.008, chosen_table_z_m=0,
             reason='板 A 是纸贴桌面；源 PnP 对象点 z=0，不把旧配准的8mm误差变成真实台阶',
             contour_annotation='head_0 上人工近似描绘可见铃外轮廓，约±2px；不是独立尺量或精确分割。桌面四角在画面外，未伪造其观测误差。')))


if __name__ == '__main__':
    main()
