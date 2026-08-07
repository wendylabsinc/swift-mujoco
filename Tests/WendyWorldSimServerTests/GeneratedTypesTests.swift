import Testing
import Foundation
@testable import WendyWorldSimServer

@Test func simRunningResponseRoundTripsThroughJSON() throws {
    // NOTE: argument order here is alphabetical (file, slot, title / focus, running), not the
    // schema-declaration order shown in the task brief. The swift-json-schema plugin
    // (JSONSchemaGeneratorCore/CodeGenerator.swift) unconditionally sorts a type's properties
    // alphabetically by JSON key before emitting struct fields/memberwise-init parameters, in
    // both 0.1.0 and 0.2.0. Swift requires call-site argument order to match declaration order,
    // so the call below is reordered to match the actual generated initializer. See
    // task-4-report.md for details.
    let value = SimRunningResponse(focus: "default",
                                   running: [RunningSim(file: "default", slot: "default", title: "Falling cube")])
    let data = try JSONEncoder().encode(value)
    let decoded = try JSONDecoder().decode(SimRunningResponse.self, from: data)
    #expect(decoded == value)
    let obj = try JSONSerialization.jsonObject(with: data) as! [String: Any]
    #expect(Set(obj.keys) == ["running", "focus"])
}

@Test func simListResponseRoundTripsThroughJSON() throws {
    let value = SimListResponse(current: nil, sims: [])
    let decoded = try JSONDecoder().decode(SimListResponse.self, from: JSONEncoder().encode(value))
    #expect(decoded == value)
}

@Test func simControlResponseRoundTripsThroughJSON() throws {
    let value = SimControlResponse(control: SimControl(paused: true, reset: nil, step: nil), ok: true)
    let decoded = try JSONDecoder().decode(SimControlResponse.self, from: JSONEncoder().encode(value))
    #expect(decoded == value)
}
