# wg-lo-filter
Control software for FINER/Tunable waveguide LO filter

## Configuration

Python 3.11 or later is required (`tomllib`). Keep `config.toml` in the
project root alongside `scripts/`, including when deploying to Raspberry Pi.
The configuration path is resolved relative to `scripts/utils.py`, not the
current working directory.

`config.toml` defines:

- `receivers.<name>.bpf_id`: receiver-to-BPF mapping.
- `receivers.<name>.cli_name`: `4+5` / `6+7`.
- `receivers.<name>.rf_to_filter_multiplier`: RF LO / filter center (2 / 3).
- `bpfs.<id>.hpf_ids`: HPFs to prepare and halt, in the listed order.
- `bpfs.<id>.move_order`: HPF motion order; each member must appear once.
- `hpfs.<id>.slave_id`: zero-based EtherCAT slave ID.
- `hpfs.<id>.cutoff_side`: `hpf` for the lower cutoff, `lpf` for the upper cutoff.
- `hpfs.<id>.A`, `B`, `X0`: coefficients for
  `position_mm = X0 - A / (cutoff_GHz - B)`.
- `hpfs.<id>.FREQ`, `FRQ2`: actuator excitation frequencies in Hz.
- `motion`: default/index velocity, acceleration/deceleration (existing controller
  command units), and encoder resolution in micrometres per encoder unit.
- `timing`: PDO exchange pauses, polling intervals, and reset/settling waits in seconds.
- `timeouts`: Python wait limits in seconds; `ethercat_state_check_us` is in microseconds.
- `pdo_settle`: exchange counts and pause interval used while awaiting state updates.
- `controller_defaults`: shared controller settings, including `LLIM`/`HLIM` in
  encoder units. Parameter transmission order is explicitly retained in the code.

The values were transferred from the existing scripts on 2026-10-04 without
retuning. When tuning a value, add the actual measurement date, conditions,
and reason beside that entry. Commit the comment and value together.
The old artificial CLI log-display pauses are omitted. Controller/PDO waits
are retained. HALT status is read after 0.5 seconds; reset status retains its
previous 0.5-second wait.

The BPF cancellation boundary retains the existing expression
`calculated_position >= MAX_DESIRED_POSITION_MM - RESOLUTION_MM`, with
`MAX_DESIRED_POSITION_MM = 2.5` in `utils.py`. With the current resolution,
positions at or above 2.49875 mm are canceled; this is separate from `HLIM`.
The legacy `command(delay=...)` argument remains unused and does not introduce
a new pause.

Configuration is loaded and validated when `scripts.utils` is imported.
Restart the script or interactive Python session after editing it.
Each HPF must belong to exactly one BPF, and slave IDs must be unique.
The existing HPF #1–#6 and B45/B67 names remain available as compatibility
constants; use the lookup functions for control logic.

## CLI構成と実行

実行用Pythonは `scripts/lo-filter.py` と `scripts/utils.py` の2ファイル。
旧個別CLIおよび `logging_utils.py` は統合済み。
TOML、テスト、依存設定は残します。

`scripts/`に移動し、次のように実行します。
uvは親ディレクトリの `pyproject.toml` を使用します。

```sh
uv run lo-filter.py prepare --rx 6+7
uv run lo-filter.py set --rx 6+7 --lo 230 --bw 4
uv run lo-filter.py read-status --rx 6+7
uv run lo-filter.py halt --rx 6+7
uv run lo-filter.py reset --rx 6+7

# HPF単体の保守操作
uv run lo-filter.py prepare --hpf-id 5
uv run lo-filter.py read-status --hpf-id 5
uv run lo-filter.py halt --hpf-id 5
uv run lo-filter.py reset --hpf-id 5
```

プロジェクトルートからは `uv run scripts/lo-filter.py ...`。
`lo-filter` という実行コマンドは登録していません。
依存は `pyproject.toml` のPySOEM・Typer。Python 3.11以上が必要です。

| コマンド | 必須引数 | 任意引数 |
| --- | --- | --- |
| prepare | --rx / --hpf-id のどちらか一方 | --log-level |
| set | --rx、--lo、--bw | --log-level |
| read-status | --rx / --hpf-id のどちらか一方 | --log-level |
| halt | --rx / --hpf-id のどちらか一方 | --log-level |
| reset | --rx / --hpf-id のどちらか一方 | --log-level |

対象指定の省略・同時指定は受け付けません。setはHPF単体指定を受け付けません。
HPF IDからslave IDはTOMLで解決します。EtherCATインターフェースは従来のeth0です。

## 周波数変換

| --rx | 受信機 | BPF ID | 逓倍係数 |
| --- | --- | --- | --- |
| 4+5 | B45 | 1 | 2 |
| 6+7 | B67 | 2 | 3 |

`--lo` はRF側第1LO周波数[GHz]。`--bw`はフィルター位置での全帯域幅[GHz]。

- フィルター中心 = lo / 逓倍係数
- 下側cutoff = 中心 - bw/2
- 上側cutoff = 中心 + bw/2

rx=6+7、lo=230、bw=4では、中心76.66667 GHz、
下側74.66667 GHz、上側78.66667 GHzです。
CLIからRF周波数を既存move_bpfの中心周波数に直接渡しません。

## 準備・駆動・停止の仕様

- prepare: 選択対象ごとにreset → 設定投入 → enable → 原点探索。
- set: 対象BPF全体のenabled、encoder_valid、既存has_errorを確認して移動。
  自動prepareはしません。準備不足はprepareを促すエラーログを出し、移動しません。
- FREQ等の読み戻し・照合や、移動中判定は追加していません。
- read-status: enable/reset/prepare命令を送らずPDO状態を取得。
  ただしmasterの初期化・終了は行うため、EtherCATバス状態が不変とは保証しません。
- halt: 指定対象へHALTを送信し、0.5秒後に状態を表示。
- reset: prepareや原点探索を行いません。次の移動前にprepareを実行してください。

prepare/setの駆動中に例外またはCtrl+Cが発生した場合、対象BPFの全3台に
内部HALTを試みます。HPF単体prepareでも、その所属BPFの3台が対象です。
CLIのhalt関数ではなく制御層の停止関数を、同じmasterから呼びます。
0.5秒後に各台のステータスをログへ出します。
HALT・状態取得に失敗しても残りの台の処理を試み、元の異常は保持します。
クローズはその後です。HALTは停止成功を保証する緊急停止機構ではありません。

引数不正・接続失敗・準備不足・ソフトリミットによる駆動前の拒否では
内部HALTは行いません。ソフトリミットの既存境界は変更していません。
途中まで動いた場合の自動ロールバックは実装していません。

## ログとFINERへの通知

全CLIで `--log-level DEBUG|INFO|WARNING|ERROR|CRITICAL` を使用できます。
大文字小文字を区別せず、既定はINFOです。
人間向けログを標準エラー出力へ出し、SSH経由でFINER側に表示します。

- INFO: 進行状況・位置・結果。read-statusおよびHALT後は全ステータスも表示。
- DEBUG: 詳細ステータス・診断・例外のTraceback。
- WARNING: 移動キャンセル・Ctrl+C等。
- ERROR: 制御・HALT失敗・準備不足等。
- CRITICAL: そのレベル以上のみ（現在専用メッセージなし）。

ログレベルを変えても制御処理は省略しません。
JSON・FINER側の結果収集／保存・独自の終了コード体系は実装しません。
CLIは制御例外をログ化して終了します。通常のOS・SSH・Python・Typerの
usageエラーの終了コード自体は存在します。

```sh
uv run lo-filter.py prepare --rx 6+7 --log-level DEBUG
uv run lo-filter.py set --rx 6+7 --lo 230 --bw 4 --log-level WARNING
```

importだけでコンソール出力・EtherCAT接続は開始しません。
IPythonでは、scripts内なら次のように明示設定します。

```python
import utils as u
u.configure_logging("DEBUG")
```

## 接続管理と逐次実行

```python
with u.ethercat_master("eth0") as master:
    u.prepare_selected(master, 2, u.get_bpf_slave_ids(2))
    u.move_bpf(master, 2, 230 / 3, 4)
# ここでクローズ済み
```

各通信・制御関数はmasterを第一引数に受け取ります。
純粋な設定・計算・単位変換関数にはmasterは不要です。
旧u.init()は使いません。with内で手動closeもしません。
正常終了・初期化失敗・例外・Ctrl+CでもINIT要求とクローズを試みます。
open失敗時はINITを省略します。終了時の例外で元の異常を置き換えません。
SIGKILL、電源断、close自体の失敗までは保証できません。

要求の直列化は呼び出し側shellで行い、Python側の排他制御は実装しません。

```sh
uv run lo-filter.py set --rx 6+7 --lo 230 --bw 4
uv run lo-filter.py set --rx 4+5 --lo 160 --bw 4
```

バックグラウンド化せず、別のshell・SSHからも同時に制御しない運用が前提です。
この2行はプロセス終了を待つだけで、成功を確認して次へ進む仕組みではありません。
別プロセスからのHALT割り込みも提供しません。

## テスト

プロジェクトルートから、実機通信なしのモックテストを実行できます。

```sh
uv run python -m unittest discover -s tests -v
```

引数・受信機とHPFの対応・逓倍・準備不足時の拒否・内部HALT・Ctrl+C・
クローズ・既存ソフトリミットと通信タイミングを検証します。
実機の駆動・SSH経由の動作は別途確認が必要です。
