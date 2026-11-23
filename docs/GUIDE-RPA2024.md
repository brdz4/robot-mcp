# Guide de suivi — vérifications RPA 2024 sur Robot

Document de travail pour dérouler, dans l'ordre, la validation du module `rpa24.py` sur un
bâtiment réel. Coche chaque étape, note les valeurs obtenues et les décisions prises.

> **Règle de base** : travailler sur une **copie** du modèle. Les outils RPA 2024 ne modifient
> rien sauf `robot_create_rpa24_combinations`, qui crée des combinaisons et n'a pas d'annulation.

---

## 0. Fiche projet à remplir avant de commencer

| Donnée | Symbole | Valeur | Source |
|---|---|---|---|
| Nom du projet / fichier | — | | |
| Zone de sismicité | I–VI | | Tableau 3.3 |
| Coefficient de zone | A | | Tableau 3.3 |
| Groupe d'importance | 1A/1B/2/3 | | Tableau 3.4 |
| Coefficient d'importance | I | | Tableau 3.4 |
| Classe de site | S1–S4 | | Tableaux 3.8/3.9 |
| Type de spectre | 1 ou 2 | | zones IV-VI = 1 ; I-III = 2 |
| Coefficient de comportement | R | | Tableau 3.18 |
| Facteur de qualité | QF | | Tableau 3.19 |
| Coefficient d'accompagnement | ψ | | Tableau 4.2 |
| Système de contreventement | — | | projet (CT, Tableau 4.3) |
| Type de structure | BA / acier / PAF / bois / maçonnerie | | projet |
| Éléments non structuraux | fragiles / ductiles | | projet |
| Béton | fc28 (MPa) | | projet |
| Acier | fe (MPa) | | projet |
| Enrobage au centre des armatures | cover (cm) | | projet |
| Barres par face en poteau | — | | projet |

### Numéros des cas de charges

| Rôle | N° du cas | Nom dans Robot | Remarque |
|---|---|---|---|
| G — permanentes | | | |
| Q — exploitation | | | |
| Ex — séisme X | | | |
| Ey — séisme Y | | | |
| Ez — séisme vertical | | | facultatif |
| Combinaison G + ψQ | | | pour le poids sismique et P-Δ |

### Facteur d'échelle des cas sismiques

| Cas | Spectre chargé | `scale` retenu | Justification |
|---|---|---|---|
| Ex | élastique / réduit | | |
| Ey | élastique / réduit | | |

> Si le spectre chargé dans Robot est **élastique**, `scale` doit l'amener au niveau de calcul.
> S'il est **déjà réduit** (R et QF inclus), `scale = 1`. Un `scale` faux fausse le cortant
> basal et les dérives.

---

## 1. Vérifier la connexion

- [ ] `robot_health`

**Attendu** : `robot_running = true`, `project_active = true`, `ready = true`.

Si `project_active = false` : ouvrir ou créer le projet dans Robot, puis recommencer.

**Noté** : ______________________________________________

---

## 2. Inventorier le modèle

- [ ] `robot_get_project_info`
- [ ] `robot_list_load_cases`
- [ ] `robot_list_storeys` (les pisos doivent exister)
- [ ] `robot_list_sections`
- [ ] `robot_list_materials`
- [ ] `robot_list_combinations` (pour éviter les doublons de noms)

**Attendu** : les cas G, Q, Ex, Ey sont identifiés ; les pisos sont définis ; les sections BA
rectangulaires sont reconnues (`CONCR_BEAM_RECT`, `CONCR_COL_R`).

**Points à vérifier** :

- [ ] nombre de niveaux ≥ 2 ;
- [ ] hauteurs de piso cohérentes ;
- [ ] pas de section sans matériau ;
- [ ] résultats disponibles (`results_status = AVAILABLE`), sinon calculer.

**Noté** : ______________________________________________

---

## 3. Contrôler le spectre

- [ ] `robot_rpa24_spectrum` avec `zone`, `site`, `group`, `r`, `qf`, `damping = 5`

Exemple de demande :

> Donne-moi le spectre RPA 2024 pour la zone III, site S2, groupe 2, R = 5.5, QF = 1.0,
> amortissement 5 %, aux périodes 0 / T1 / T2 / T3 / 1 s / 2 s.

**Attendu** : `parameters` avec `A`, `I`, `S`, `T1`, `T2`, `T3`, `B`, `k`, `floor_sa_g`, puis
`points` avec `elastic_sa_g` et `design_sa_g`.

**Contrôle** :

- [ ] `design_sa_g` à `T2` = `B · 2.5 · QF / R` ;
- [ ] `design_sa_g` ≥ `0.2 · A · I` partout ;
- [ ] le spectre chargé dans Robot donne les mêmes valeurs (à la tolérance près).

| Période | Robot (g) | Outil (g) | Écart |
|---|---|---|---|
| T1 | | | |
| T2 | | | |
| T3 | | | |
| T période fondamentale | | | |

**Conclusion** : [ ] cohérent  [ ] écart à corriger avant de continuer

---

## 4. Irrégularités et facteur de qualité

- [ ] `robot_check_rpa24_irregularities` avec `qf_category` = `a`, `b` ou `c`

Données à fournir si disponibles :

- [ ] `e0x_m`, `e0y_m` (excentricité structurale) et `rx_m`, `ry_m` (rayon de torsion) ;
- [ ] `length_x_m`, `width_y_m` (sinon enveloppe des nœuds) ;
- [ ] `recess_ratio` (parties rentrantes) ;
- [ ] `opening_ratio` (ouvertures de plancher) ;
- [ ] `mass_variation_ratio` ou `mass_case` ;
- [ ] `stiffness_variation_ratio` ;
- [ ] `bays_x`, `bays_y`.

**Attendu** : `criteria` avec `OK` / `FAIL` / `UNKNOWN`, `groups`, `qf_suggested_min` et
`qf_suggested_max`.

| Critère | Valeur | Limite | Statut |
|---|---|---|---|
| Torsion X | | 0.2·rx | |
| Torsion Y | | 0.2·ry | |
| L/B | | 4 | |
| Parties rentrantes | | 25 % | |
| Ouvertures | | 15 % | |
| Variation de masse | | 25 % | |
| Variation de rigidité | | 25 % | |
| Niveaux | | ≥ 2 | |
| Travées | | ≥ 2 | |

**QF retenu** : ________ (justification : ______________________________)

- [ ] vérifier la torsion accidentelle de 5 % dans le modèle (§4.2.7).

---

## 5. Créer les combinaisons sismiques

- [ ] `robot_create_rpa24_combinations` avec `dry_run = true`

Paramètres :

- [ ] `dead` = cas G ;
- [ ] `live` = cas Q ;
- [ ] `seismic_x` / `seismic_y` = cas spectraux, appariés par position ;
- [ ] `occupancy` ou `psi` ;
- [ ] `foundation = true` si fondations communes ;
- [ ] `vertical = true` + un seul `vertical_z` si composante verticale requise.

**Attendu** : `dry_run = true`, `to_create` avec les noms et les facteurs, aucune modification.

**Contrôle du plan** :

- [ ] nombre de combinaisons ≤ 60 ;
- [ ] noms uniques (préfixe `RPA24`) ;
- [ ] type `ALS`, nature `SEISMIC` ;
- [ ] `E1 = ±Ex ± 0.3Ey` et `E2 = ±0.3Ex ± Ey` présents ;
- [ ] `± 1.4·E1/E2` si `foundation = true` ;
- [ ] `E3/E4/E5` si `vertical = true`.

- [ ] accord de l'utilisateur obtenu
- [ ] rappeler l'outil avec `dry_run = false`
- [ ] vérifier avec `robot_list_combinations` que les combinaisons existent

**Noté** : nombre créé = ________ ; noms = ______________________________

---

## 6. Calculer

- [ ] `robot_run_calculation` (si l'outil est disponible et l'utilisateur d'accord), sinon
      demander le calcul dans Robot.

**Attendu** : `results_status = AVAILABLE` (ou `robot_health` le confirme).

- [ ] recalcul terminé sans message bloquant ;
- [ ] résultats non `OUT_OF_DATE`.

---

## 7. Cortant basal

- [ ] `robot_check_rpa24_base_shear` avec :

```
seismic_cases = [{case: Ex, scale: ...}, {case: Ey, scale: ...}]
gravity_case  = n° de la combinaison G + ψQ (sans majoration)
zone, site, group, r, qf, system
period_s      = période connue, sinon omis (formule empirique)
direction     = AUTO
```

**Attendu** : `parameters` (`W_kn`, `hN_m`, `To_used_s`, `lambda`, `Sa_g`, `V_static_kn`),
`cases` (`V_dynamic_kn`, `ratio`, `passes`, `required_scale`), `summary`.

| Cas | V dynamique (kN) | V statique (kN) | Ratio | ≥ 0.80 ? | `required_scale` |
|---|---|---|---|---|---|
| Ex | | | | | |
| Ey | | | | | |

**Si un ratio < 0.80** :

- [ ] appliquer `summary.required_scale` à toutes les réponses sismiques (ou recharger le
      spectre avec ce facteur) ;
- [ ] recalculer ;
- [ ] répéter la vérification.

**Noté** : ______________________________________________

---

## 8. Dérives inter-étages

- [ ] `robot_check_rpa24_drifts` avec les cas sismiques et leur `scale`, `r`, `qf`,
      `structure_type`, `nonstructural`, `damage = true`

**Attendu** : `storeys` par piso et par cas, `summary.noncollapse`, `summary.damage`.

| Piso | Cas | Dérive de calcul | Limite | Statut | Source |
|---|---|---|---|---|---|
| | | | 0.0150·hk (BA) | | table / ligne |
| | | | | | |

**Contrôles** :

- [ ] `summary.noncollapse.passes = true` ;
- [ ] `summary.damage.passes = true` (si demandé) ;
- [ ] si `source = "frame_line"`, examiner le diaphragme et les planchers flexibles.

**Si échec** : augmenter la rigidité (voiles, sections), réduire la masse, corriger le modèle.

**Noté** : ______________________________________________

---

## 9. Effet P-Δ

- [ ] `robot_check_rpa24_p_delta` avec `seismic_cases = "Ex Ey"` (ou `"5 6"`) et
      `gravity_case` = G + ψQ

**Attendu** : `storeys` avec `theta`, `classification`, `amplification`, `passes` ;
`summary.max_theta`.

| Piso | Cas | θk | Classe | Amplification |
|---|---|---|---|---|
| | | | | |

**Interprétation** :

- [ ] θ < 0.10 → négligeable ;
- [ ] 0.10 ≤ θ < 0.20 → amplifier par `1/(1−θ)` ;
- [ ] θ ≥ 0.20 → structure instable, redimensionner.

**Noté** : ______________________________________________

---

## 10. Ferraillage BAEL 91 / CBA 93

- [ ] `robot_check_rpa24_rc_members` avec les **combinaisons ALS** créées, `fc28_mpa`,
      `fe_mpa`, `zone`, `cover_cm`, `bars_per_face`, et éventuellement `structural_type`.

Exemple :

> Ferraille les poteaux avec les combinaisons RPA2401 à RPA2420. fc28 = 25 MPa, fe = 400 MPa,
> zone III, enrobage 5 cm, 3 barres par face.

**Attendu** : `materials`, `summary` (`checked`, `failing`, `skipped`), `items`.

Pour chaque élément :

- [ ] poutre : `as_top_cm2`, `as_bottom_cm2`, `rho_top`, `rho_bottom`, `shear.av_s_cm2_m` ;
- [ ] poteau : `rho_required`, `as_required_cm2`, `governs`, `short_column`, `shear_y/z` ;
- [ ] `problems` : chaque ligne est une action à mener.

| Élément | Type | As/ρ requis | ρ min | ρ max | Statut | Problème |
|---|---|---|---|---|---|---|
| | | | | 4 % | | |
| | | | | | | |

**Contrôles complémentaires (hors outil)** :

- [ ] poteau fort / poutre faible ;
- [ ] zones critiques et confinement (§7.4) ;
- [ ] poteaux courts (§7.4.3) ;
- [ ] espacement des cadres ;
- [ ] recouvrements et ancrages (BAEL).

**Noté** : ______________________________________________

---

## 11. Rapport de synthèse

Remplir le tableau final et le joindre au projet.

| Vérification | Valeur | Limite | Statut | Action |
|---|---|---|---|---|
| Spectre cohérent | | — | | |
| QF | | ≥ 1 | | |
| Cortant basal | ratio | ≥ 0.80 | | |
| Dérives (non-effondrement) | | 0.0150·hk | | |
| Dérives (dommages) | | 0.0050/0.0075·hk | | |
| P-Δ | θ | < 0.10 / < 0.20 | | |
| Poutres (flexion) | μ | ≤ μlim | | |
| Poteaux (ρ) | | 0.8–4 % | | |
| Poteaux courts | λg | ≥ 5 | | |

**Hypothèses retenues** : ______________________________________________

**Décisions / actions** : ______________________________________________

**Validation ingénieur** : nom ______________ date ______________

---

## 12. Clôture

- [ ] sauvegarder le modèle (`robot_save_project`) ou une copie ;
- [ ] archiver ce guide rempli avec le projet ;
- [ ] signaler les anomalies du module (section suivante).

---

## Journal de validation

| Date | Étape | Résultat | Remarque |
|---|---|---|---|
| | | | |
| | | | |
| | | | |

---

## Dépannage

| Symptôme | Cause probable | Action |
|---|---|---|
| `project_active = false` | aucun projet ouvert | ouvrir/créer un projet dans Robot |
| `ready = false` | Robot démarre depuis < 40 s | attendre puis réessayer |
| `results_status = OUT_OF_DATE` | modèle modifié après calcul | relancer le calcul |
| Poids sismique nul | `gravity_case` sans G ou mauvaise combinaison | utiliser G + ψQ non majoré |
| `To_used_s` plafonné | période fournie > 1.3·To empirique | vérifier la période, le plafond est normal |
| Ratio de cortant < 0.80 | spectre sous-dimensionné | appliquer `required_scale`, recalculer |
| Dérive élevée sur `frame_line` | plancher non rigide / ouvertures | examiner la modélisation |
| `rho_required = null` | même 4 % insuffisant | augmenter la section ou fc28 |
| Sections ignorées (`skipped`) | section non rectangulaire BA | traiter à part |
| Combinaisons refusées | nom existant ou > 60 | changer le préfixe, réduire les cas |
| `discard_results` demandé | le changement invalide les résultats | accepter explicitement ou recalculer |

---

## Limites à rappeler dans le rapport

- Centre de rigidité et rayons de torsion non calculés : fournis ou estimés sur le modèle 3D.
- Pas de dimensionnement par capacité, ni vérification des nœuds, ni confinement détaillé.
- Pas d'analyse de sol, de liquéfaction ni d'interaction sol-structure.
- Les résultats sont des aides au calcul et doivent être validés par un ingénieur structure.
