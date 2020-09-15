#ifndef SCENARIO_RUNNER__SYNTAX__SPEED_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__SPEED_ACTION_HPP_

#include <scenario_runner/syntax/rule.hpp>
#include <scenario_runner/syntax/speed_action_target.hpp>
#include <scenario_runner/syntax/transition_dynamics.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== SpeedAction ==========================================================
   *
   * <xsd:complexType name="SpeedAction">
   *   <xsd:all>
   *     <xsd:element name="SpeedActionDynamics" type="TransitionDynamics"/>
   *     <xsd:element name="SpeedActionTarget" type="SpeedActionTarget"/>
   *   </xsd:all>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct SpeedAction
  {
    Scope inner_scope;

    const TransitionDynamics speed_action_dynamics;

    const SpeedActionTarget speed_action_target;

    template <typename Node>
    explicit SpeedAction(const Node& node, Scope& outer_scope)
      : inner_scope           { outer_scope }
      , speed_action_dynamics { readElement<TransitionDynamics>("SpeedActionDynamics", node, inner_scope) }
      , speed_action_target   { readElement<SpeedActionTarget> ("SpeedActionTarget",   node, inner_scope) }
    {}

    std::unordered_map<std::string, Boolean> accomplishments;

    auto start() // XXX UGLY CODE
    {
      if (speed_action_target.is<AbsoluteTargetSpeed>())
      {
        for (const auto& each : inner_scope.actors)
        {
          accomplishments.emplace(each, false);

          switch (speed_action_dynamics.dynamics_shape)
          {
          case DynamicsShape::linear:
            // inner_scope.connection->entity->setTargetSpeed(each, speed_action_target.as<AbsoluteTargetSpeed>().value, true);
            break;

          case DynamicsShape::step:
            // {
            //   auto status { inner_scope.getEntityStatus(each) };
            //   status.twist.linear.x = speed_action_target.as<AbsoluteTargetSpeed>().value;
            //
            //   inner_scope.connection->entity->setEntityStatus(each, status);
            //   inner_scope.connection->entity->setTargetSpeed(each, status.twist.linear.x, true);
            // }
            break;

          default:
            THROW(ImplementationFault);
          }
        }

        return unspecified;
      }
      else
      {
        THROW(ImplementationFault);
      }
    }

    auto accomplished()
    {
      if (speed_action_target.is<AbsoluteTargetSpeed>())
      {
        for (auto&& each : accomplishments)
        {
          if (not cdr(each)) try
          {
            // cdr(each) =
            //   Rule(Rule::equalTo)(
            //     inner_scope.getEntityStatus(car(each)).twist.linear.x,
            //     speed_action_target.as<AbsoluteTargetSpeed>().value);
          }
          catch (const SemanticError&) // XXX DIRTY HACK!!!
          {
            // NOTE maybe lane-changing
            cdr(each) = false;
          }
        }

        return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
      }
      else
      {
        THROW(ImplementationFault);
      }
    }

  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__SPEED_ACTION_HPP_
